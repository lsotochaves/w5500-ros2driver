#!/usr/bin/env python3
import subprocess

print("=== OpenCoRoCo Setup Menu ===")

# --- Microcontroller type selection ---
while True:
    try:
        is_f3 = int(input("Are you using an STM32F3 microcontroller? (0: No / 1: Yes): "))
        if is_f3 in [0, 1]:
            break
        print("Invalid input. Please enter 0 or 1.")
    except ValueError:
        print("Invalid input. Please enter a number (0 or 1).")

# ---------------------------------------------------------------------------
# --- STM32F3 branch: ask for opmode, sistema, and plane
# ---------------------------------------------------------------------------
if is_f3 == 1:
    print("\n=== STM32F3 Setup ===")

    # --- Mode selection ---
    while True:
        try:
            opmode = int(input("Select operation mode (0: voltage / 1: force): "))
            if opmode in [0, 1]:
                break
            print("Invalid input. Please enter 0 or 1.")
        except ValueError:
            print("Invalid input. Please enter a number (0 or 1).")

    sistema = None
    plane = None

    if opmode == 1:
        # --- System selection ---
        while True:
            try:
                sistema = int(input("Select system (0: 10 kg / 1: 200 kg): "))
                if sistema in [0, 1]:
                    break
                print("Invalid input. Please enter 0 or 1.")
            except ValueError:
                print("Invalid input. Please enter a number (0 or 1).")

        # --- Plane selection ---
        while True:
            try:
                plane = int(input("Select plane (0: front / 1: left / 2: back / 3: right): "))
                if 0 <= plane <= 3:
                    break
                print("Invalid input. Please enter 0, 1, 2, or 3.")
            except ValueError:
                print("Invalid input. Please enter a number (0–3).")

    # --- Build STM32F3-specific launch command ---
    launch_cmd = [
        "ros2", "launch", "w5500_ros2driver", "w5500_f3_multi.launch.py",
        f"opmode:={opmode}"
    ]
    if sistema is not None:
        launch_cmd.append(f"sistema:={sistema}")
    if plane is not None:
        launch_cmd.append(f"plane:={plane}")

# ---------------------------------------------------------------------------
# --- STM32F4 (or other) branch: run the default multi-launch file
# ---------------------------------------------------------------------------
else:
    print("\nLaunching default multi-sensor (STM32F4) configuration...")
    launch_cmd = [
        "ros2", "launch", "w5500_ros2driver", "w5500_multi.launch.py"
    ]

# ---------------------------------------------------------------------------
# --- Execute command
# ---------------------------------------------------------------------------
print("\nLaunching ROS 2 node with:")
print(" ".join(launch_cmd))
print("--------------------------------------")

try:
    subprocess.run(launch_cmd)
except KeyboardInterrupt:
    print("\nLaunch interrupted by user.")
