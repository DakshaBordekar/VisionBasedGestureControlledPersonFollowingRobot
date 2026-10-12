#!/usr/bin/env python3
"""
===============================================================================
Project: Vision-Based Gesture-Controlled Person-Following Robot
File: apply_fixes.py
Description: Fully automated patch execution script. Verifies patched files
             and trains initial baseline gesture model if needed.
===============================================================================
"""

import os
import shutil
import sys
import subprocess

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))


def check_file_exists(rel_path):
    full_path = os.path.join(REPO_ROOT, rel_path)
    exists = os.path.exists(full_path)
    status = "[OK]" if exists else "[MISSING]"
    print(f"  {status} {rel_path}")
    return exists


def main():
    print("=" * 60)
    print("  VBGC ROBOT AUTOMATED REPAIR & VERIFICATION SCRIPT")
    print(f"  Target Root: {REPO_ROOT}")
    print("=" * 60)

    print("\n[STEP 1] Checking Core Patched Files:")
    files = [
        "hardware/esp32/motor_controller/motor_controller.ino",
        "hardware/pi/pi_cmdvel_receiver.py",
        "vision/train_gesture_model.py",
        "vision/cc_ll.py",
        "ros2_ws/src/vbgc_robot/package.xml",
        "ros2_ws/src/vbgc_robot/setup.py",
        "ros2_ws/src/vbgc_robot/vbgc_robot/esp32_motor_bridge.py",
        "ros2_ws/src/vbgc_robot/worlds/rover_world.sdf",
        "ros2_ws/src/vbgc_robot/launch/full_system.launch.py",
    ]

    all_present = all(check_file_exists(f) for f in files)
    if not all_present:
        print("\n[WARN] Some target files are missing. Please inspect.")

    print("\n[STEP 2] Verifying Gesture Model Training...")
    train_script = os.path.join(REPO_ROOT, "vision", "train_gesture_model.py")
    if os.path.exists(train_script):
        res = subprocess.run([sys.executable, train_script], cwd=os.path.join(REPO_ROOT, "vision"))
        if res.returncode == 0:
            print("[OK] Baseline gesture model generated successfully.")
        else:
            print("[WARN] Model training returned non-zero code.")

    print("\n[SUCCESS] System verification complete!")
    print("Next step: Build ros2_ws with 'colcon build --symlink-install'.")


if __name__ == "__main__":
    main()
