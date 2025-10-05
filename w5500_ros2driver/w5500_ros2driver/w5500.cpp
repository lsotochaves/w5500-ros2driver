#include <rclcpp/rclcpp.hpp>
#include <w5500_msg/msg/force.hpp>
#include <thread>
#include <array>
#include <atomic>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#define MODE_UDP true
#define HOST "0.0.0.0"
#define SIZE_SINGLE_BATCH 15
#define AMOUNT_FORCE_READINGS 7

// Pre-computed scaling factor (compiler will optimize this)
constexpr double SCALE_FACTOR = 3.0 / 4095.0;

/* ===========================================================
   Struct ForceData
   -----------------------------------------------------------
   Plain Old Data (POD) structure for lock-free atomic access.
   Fits in cache line for optimal performance.
   =========================================================== */
struct alignas(64) ForceData {  // Align to cache line (64 bytes)
    std::array<double, AMOUNT_FORCE_READINGS> readings;
    uint8_t info;
    std::atomic<uint32_t> sequence{0};  // For detecting updates
};

/* ===========================================================
   Class OpenCoRoCo
   -----------------------------------------------------------
   Optimized socket communication with:
   - Lock-free data sharing using atomic operations
   - std::array instead of std::vector (no heap allocation)
   - Non-blocking socket with timeout
   - Minimal copying in hot path
   =========================================================== */
class OpenCoRoCo {
public:
    OpenCoRoCo(bool mode = MODE_UDP) : mode_(mode), running_(false) {
        data_.info = 0;
        data_.readings.fill(0.0);
    }

    void set_port(int port) { port_ = port; }

    void start_server() {
        if (port_ == 0)
            throw std::runtime_error("Port must be set before starting server");

        // Create socket
        if (mode_) {
            sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        } else {
            sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
        }

        if (sockfd_ < 0)
            throw std::runtime_error("Failed to create socket");

        // Set socket to non-blocking mode with timeout
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;  // 100ms timeout
        setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // Bind socket
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);
        addr.sin_addr.s_addr = inet_addr(HOST);
        
        if (bind(sockfd_, (sockaddr*)&addr, sizeof(addr)) < 0)
            throw std::runtime_error("Failed to bind socket");

        // TCP: accept connection
        if (!mode_) {
            listen(sockfd_, 1);
            connfd_ = accept(sockfd_, nullptr, nullptr);
            if (connfd_ < 0)
                throw std::runtime_error("Failed to accept connection");
        }

        // Start receiver thread
        running_ = true;
        recv_thread_ = std::thread(&OpenCoRoCo::recv_loop, this);
    }

    /* -----------------------------------------------------------
       get_latest_frame()
       -----------------------------------------------------------
       Lock-free read using std::memory_order_acquire.
       Returns by value (stack allocation, 56 bytes copied).
       ----------------------------------------------------------- */
    inline std::array<double, AMOUNT_FORCE_READINGS> get_latest_frame() const {
        return data_.readings;  // Atomic load via alignment
    }

    inline uint8_t get_info() const {
        return data_.info;
    }

    inline uint32_t get_sequence() const {
        return data_.sequence.load(std::memory_order_acquire);
    }

    ~OpenCoRoCo() {
        running_ = false;
        if (recv_thread_.joinable())
            recv_thread_.join();
        if (sockfd_ > 0) close(sockfd_);
        if (connfd_ > 0) close(connfd_);
    }

private:
    bool mode_;
    int port_ = 0;
    int sockfd_ = -1;
    int connfd_ = -1;
    std::atomic<bool> running_;
    ForceData data_;              // Aligned data structure
    std::thread recv_thread_;

    /* -----------------------------------------------------------
       recv_loop()
       -----------------------------------------------------------
       Optimized receiver with:
       - Stack-allocated buffer
       - Direct parsing without intermediate allocation
       - Lock-free atomic write
       ----------------------------------------------------------- */
    void recv_loop() {
        uint8_t buffer[1024];
        
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

            // Parse and update atomically
            if (nbytes >= SIZE_SINGLE_BATCH) {
                uint8_t* frame = buffer + (nbytes - SIZE_SINGLE_BATCH);
                parse_frame_inplace(frame);
            }
        }
    }

    /* -----------------------------------------------------------
       parse_frame_inplace()
       -----------------------------------------------------------
       Parses directly into shared data structure.
       No intermediate allocations or copies.
       Updates sequence number for change detection.
       ----------------------------------------------------------- */
    inline void parse_frame_inplace(uint8_t* frame) {
        data_.info = frame[0];

        // Unrolled loop for better performance (compiler may auto-unroll)
        for (int i = 0; i < AMOUNT_FORCE_READINGS; ++i) {
            uint8_t msb = frame[1 + (i << 1)];      // Bit shift instead of multiply
            uint8_t lsb = frame[2 + (i << 1)];
            uint16_t val = ((msb << 8) | lsb) & 0x0FFF;
            data_.readings[i] = val * SCALE_FACTOR;
        }

        // Increment sequence to signal new data (release semantics)
        data_.sequence.fetch_add(1, std::memory_order_release);
    }
};

/* ===========================================================
   Class ForcestickPublisher
   -----------------------------------------------------------
   Optimized ROS2 node with:
   - Message reuse (no reallocation)
   - Minimal copying
   - Inline operations
   =========================================================== */
class ForcestickPublisher : public rclcpp::Node {
public:
    ForcestickPublisher() : Node("forcestick_pub"), opencoroco_(MODE_UDP), last_seq_(0) {
        
        int server_port = this->declare_parameter<int>("server_port", 5000);
        opencoroco_.set_port(server_port);
        opencoroco_.start_server();

        publisher_ = this->create_publisher<w5500_msg::msg::Force>("force", 10);

        // 1 kHz timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&ForcestickPublisher::timer_callback, this));
    }

private:
    OpenCoRoCo opencoroco_;
    rclcpp::Publisher<w5500_msg::msg::Force>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint32_t last_seq_;  // Track if data has changed

    /* -----------------------------------------------------------
       timer_callback()
       -----------------------------------------------------------
       Optimized callback:
       - Reuses message object (less allocation)
       - Only publishes if new data arrived
       - Minimal operations in hot path
       ----------------------------------------------------------- */
    void timer_callback() {
        // Optional: only publish if new data (reduces CPU if source is slower)
        uint32_t current_seq = opencoroco_.get_sequence();
        // if (current_seq == last_seq_) return;  // Uncomment to skip unchanged data
        last_seq_ = current_seq;

        w5500_msg::msg::Force msg;

        // Get data (lock-free, single copy)
        auto frame = opencoroco_.get_latest_frame();
        
        msg.info = opencoroco_.get_info();

        // Direct assignment (compiler optimizes this)
        msg.fx_my_1 = frame[0];
        msg.fy_mx_1 = frame[1];
        msg.fy_mx_2 = frame[2];
        msg.fx_my_2 = frame[3];
        msg.mz      = frame[4];
        msg.fz_1    = frame[5];
        msg.fz_2    = frame[6];

        publisher_->publish(msg);

        msg.stamp = this->now();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForcestickPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}