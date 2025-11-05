#!/usr/bin/env python3
import subprocess

print("=== STM32F3 Setup Menu ===")

# --- Mode selection ---
while True:
    try:
        opmode = int(input("Select operation mode (0: voltage / 1: force): "))
        if opmode in [0, 1]:
            break
        print("Invalid input. Please enter 0 or 1.")
    except ValueError:
        print("Invalid input. Please enter a number (0 or 1).")

# --- Conditional setup ---
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

    # --- Plane selection (integer only) ---
    while True:
        try:
            plane = int(input("Select plane (0: front / 1: left / 2: back / 3: right): "))
            if 0 <= plane <= 3:
                break
            print("Invalid input. Please enter 0, 1, 2, or 3.")
        except ValueError:
            print("Invalid input. Please enter a number (0–3).")

# --- Build launch command ---
launch_cmd = [
    "ros2", "launch", "w5500_ros2driver", "w5500_multi.launch.py",
    f"opmode:={opmode}"
]

# Only append optional ones if defined
if sistema is not None:
    launch_cmd.append(f"sistema:={sistema}")
if plane is not None:
    launch_cmd.append(f"plane:={plane}")

print("\nLaunching ROS 2 node with:")
print(" ".join(launch_cmd))
print("--------------------------------------")

try:
    subprocess.run(launch_cmd)
except KeyboardInterrupt:
    print("\nLaunch interrupted by user.")
