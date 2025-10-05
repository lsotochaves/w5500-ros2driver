#include <rclcpp/rclcpp.hpp>
#include <w5500_msg/msg/force.hpp>
#include <thread>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <chrono>

using namespace std::chrono_literals;

#define MODE_UDP true
#define HOST "0.0.0.0"
#define SIZE_SINGLE_BATCH 15
#define AMOUNT_FORCE_READINGS 7

constexpr double SCALE_FACTOR = 3.0 / 4095.0;

/* ===========================================================
   Struct ForceData  (cache aligned)
   -----------------------------------------------------------
   Shared buffer between threads — no locks, only atomic seq.
   =========================================================== */
struct alignas(64) ForceData {
    std::array<double, AMOUNT_FORCE_READINGS> readings;
    uint8_t info;
    std::atomic<uint32_t> sequence{0};
};

/* ===========================================================
   Class OpenCoRoCo
   -----------------------------------------------------------
   Handles UDP/TCP socket reception with minimal overhead.
   =========================================================== */
class OpenCoRoCo {
public:
    explicit OpenCoRoCo(bool mode = MODE_UDP)
        : mode_(mode), running_(false), logger_(rclcpp::get_logger("OpenCoRoCo"))
    {
        data_.info = 0;
        data_.readings.fill(0.0);
    }

    void set_port(int port) { port_ = port; }
    void set_rt_config(int cpu_core, int sleep_us) {
        cpu_core_ = cpu_core;
        sleep_us_ = sleep_us;
    }

    void start_server() {
        if (port_ == 0)
            throw std::runtime_error("Port must be set before starting server");

        sockfd_ = socket(AF_INET, mode_ ? SOCK_DGRAM : SOCK_STREAM, 0);
        if (sockfd_ < 0)
            throw std::runtime_error("Failed to create socket");

        int yes = 1;
        setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        int bufsize = 256 * 1024;
        setsockopt(sockfd_, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize));

        int flags = fcntl(sockfd_, F_GETFL, 0);
        fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);
        addr.sin_addr.s_addr = inet_addr(HOST);

        if (bind(sockfd_, (sockaddr*)&addr, sizeof(addr)) < 0)
            throw std::runtime_error("Failed to bind socket");

        if (!mode_) {
            listen(sockfd_, 1);
            RCLCPP_INFO(logger_, "Waiting for TCP connection...");
            connfd_ = accept(sockfd_, nullptr, nullptr);
            if (connfd_ < 0)
                throw std::runtime_error("Failed to accept TCP connection");
            int cflags = fcntl(connfd_, F_GETFL, 0);
            fcntl(connfd_, F_SETFL, cflags | O_NONBLOCK);
            RCLCPP_INFO(logger_, "TCP client connected");
        }

        running_ = true;
        recv_thread_ = std::thread(&OpenCoRoCo::recv_loop, this);
    }

    inline const std::array<double, AMOUNT_FORCE_READINGS>& get_latest_frame() const noexcept {
        return data_.readings;
    }

    inline uint8_t get_info() const noexcept { return data_.info; }

    inline uint32_t get_sequence() const noexcept {
        return data_.sequence.load(std::memory_order_acquire);
    }

    ~OpenCoRoCo() {
        running_ = false;
        if (recv_thread_.joinable()) recv_thread_.join();
        if (sockfd_ > 0) close(sockfd_);
        if (connfd_ > 0) close(connfd_);
    }

private:
    bool mode_;
    int port_ = 0;
    int cpu_core_ = -1;
    int sleep_us_ = 100;
    int sockfd_ = -1;
    int connfd_ = -1;
    std::atomic<bool> running_;
    ForceData data_;
    std::thread recv_thread_;
    rclcpp::Logger logger_;

    void set_realtime_priority() {
        sched_param param{};
        param.sched_priority = 80;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            RCLCPP_WARN(logger_,
                        "Failed to set RT priority (need CAP_SYS_NICE). "
                        "Run: sudo setcap cap_sys_nice=eip <executable>");
        } else {
            RCLCPP_INFO(logger_, "Real-time priority enabled (SCHED_FIFO, prio=80)");
        }

        if (cpu_core_ >= 0) {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(cpu_core_, &cpuset);
            if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0)
                RCLCPP_WARN(logger_, "Failed to set CPU affinity to core %d", cpu_core_);
            else
                RCLCPP_INFO(logger_, "Receiver pinned to CPU core %d", cpu_core_);
        }
    }

    void recv_loop() {
        set_realtime_priority();
        uint8_t buffer[1024];
        const auto sleep_duration = std::chrono::microseconds(sleep_us_);

        while (running_) {
            ssize_t nbytes = 0;

            if (mode_) {
                sockaddr_in src{};
                socklen_t len = sizeof(src);
                nbytes = recvfrom(sockfd_, buffer, sizeof(buffer), 0,
                                  (sockaddr*)&src, &len);
            } else {
                nbytes = recv(connfd_, buffer, sizeof(buffer), 0);
            }

            if (nbytes < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    std::this_thread::sleep_for(sleep_duration);
                    continue;
                }
                continue;
            }

            if (nbytes == 0 && !mode_) {
                RCLCPP_WARN(logger_, "TCP connection closed");
                break;
            }

            if (nbytes >= SIZE_SINGLE_BATCH) {
                uint8_t* frame = buffer + (nbytes - SIZE_SINGLE_BATCH);
                parse_frame_inplace(frame);
            }
        }
    }

    inline void parse_frame_inplace(uint8_t* frame) noexcept {
        data_.info = frame[0];
        for (int i = 0; i < AMOUNT_FORCE_READINGS; ++i) {
            uint8_t msb = frame[1 + (i << 1)];
            uint8_t lsb = frame[2 + (i << 1)];
            uint16_t val = ((msb << 8) | lsb) & 0x0FFF;
            data_.readings[i] = val * SCALE_FACTOR;
        }
        data_.sequence.fetch_add(1, std::memory_order_release);
    }
};

/* ===========================================================
   Class ForcestickPublisher
   -----------------------------------------------------------
   Publishes at 1 kHz with minimal copying.
   =========================================================== */
class ForcestickPublisher : public rclcpp::Node {
public:
    ForcestickPublisher()
        : Node("forcestick_pub"), opencoroco_(MODE_UDP), last_seq_(0)
    {
        int server_port = this->declare_parameter<int>("server_port", 5000);
        int cpu_core = this->declare_parameter<int>("cpu_core", -1);
        int recv_sleep_us = this->declare_parameter<int>("recv_sleep_us", 100);
        bool skip_unchanged = this->declare_parameter<bool>("skip_unchanged", false);
        skip_unchanged_ = skip_unchanged;

        opencoroco_.set_port(server_port);
        opencoroco_.set_rt_config(cpu_core, recv_sleep_us);
        opencoroco_.start_server();

        publisher_ = this->create_publisher<w5500_msg::msg::Force>("force", 10);

        // --- Wall timer (Humble compatible) ---
        timer_ = this->create_wall_timer(
            1ms, std::bind(&ForcestickPublisher::loop, this));

        RCLCPP_INFO(this->get_logger(),
                    "ForcestickPublisher running @1kHz on port=%d (skip_unchanged=%s)",
                    server_port, skip_unchanged ? "true" : "false");
    }

private:
    OpenCoRoCo opencoroco_;
    rclcpp::Publisher<w5500_msg::msg::Force>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint32_t last_seq_;
    bool skip_unchanged_;

    void loop() {
        uint32_t current_seq = opencoroco_.get_sequence();
        if (skip_unchanged_ && current_seq == last_seq_) return;
        last_seq_ = current_seq;

        w5500_msg::msg::Force msg;
        const auto& frame = opencoroco_.get_latest_frame();
        msg.info = opencoroco_.get_info();

        // Convert time to builtin_interfaces::msg::Time manually (Humble-safe)
        auto now = this->get_clock()->now();
        msg.stamp.sec = now.seconds();
        msg.stamp.nanosec = now.nanoseconds() % 1000000000ull;

        msg.fx_my_1 = frame[0];
        msg.fy_mx_1 = frame[1];
        msg.fy_mx_2 = frame[2];
        msg.fx_my_2 = frame[3];
        msg.mz      = frame[4];
        msg.fz_1    = frame[5];
        msg.fz_2    = frame[6];

        publisher_->publish(msg);
    }
};

/* ===========================================================
   main()
   =========================================================== */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForcestickPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
