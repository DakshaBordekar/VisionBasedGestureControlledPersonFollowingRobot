#!/usr/bin/env python3
"""
===============================================================================
Project: Vision-Based Gesture-Controlled Person-Following Robot
Target: Raspberry Pi 4 Model B (Raspberry Pi OS 64-bit)
File: hardware/pi/pi_cmdvel_receiver.py
Description: Dedicated TCP-to-Serial bridge server daemon. Listens on port 5000
             for velocity commands transmitted from laptop ROS 2 bridge and
             forwards them to the connected ESP32 over local serial UART.
===============================================================================
"""

import socket
import serial
import time
import sys
import threading
import signal

HOST = "0.0.0.0"
PORT = 5000
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
WATCHDOG_TIMEOUT = 0.5  # seconds

running = True
serial_lock = threading.Lock()
last_cmd_time = 0.0


def signal_handler(sig, frame):
    global running
    print("\n[INFO] Termination signal received. Halting receiver...")
    running = False
    sys.exit(0)


def create_serial_connection():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"[OK] Opened serial interface {SERIAL_PORT} @ {BAUD_RATE} baud.")
        return ser
    except serial.SerialException as exc:
        print(f"[ERROR] Could not open {SERIAL_PORT}: {exc}")
        return None


def watchdog_thread(ser):
    global running, last_cmd_time
    while running:
        time.sleep(0.1)
        if last_cmd_time > 0 and (time.time() - last_cmd_time > WATCHDOG_TIMEOUT):
            with serial_lock:
                if ser and ser.is_open:
                    try:
                        ser.write(b"0.000 0.000\n")
                    except serial.SerialException:
                        pass
            last_cmd_time = 0.0


def main():
    global running, last_cmd_time
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    ser = create_serial_connection()
    wd = threading.Thread(target=watchdog_thread, args=(ser,), daemon=True)
    wd.start()

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server_sock.bind((HOST, PORT))
        server_sock.listen(1)
        print(f"[OK] TCP Bridge Server listening on {HOST}:{PORT}")
    except OSError as err:
        print(f"[FATAL] Failed to bind TCP socket: {err}")
        if ser:
            ser.close()
        return

    while running:
        print("[INFO] Waiting for connection from laptop ROS 2 node...")
        try:
            client_sock, client_addr = server_sock.accept()
            print(f"[OK] Connected by client at {client_addr}")
            buffer = ""

            while running:
                data = client_sock.recv(1024)
                if not data:
                    print("[WARN] Client disconnected.")
                    break

                buffer += data.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    last_cmd_time = time.time()
                    with serial_lock:
                        if ser and ser.is_open:
                            try:
                                ser.write((line + "\n").encode("utf-8"))
                            except serial.SerialException as se:
                                print(f"[ERROR] Serial write failure: {se}")

        except (socket.error, ConnectionResetError) as e:
            print(f"[WARN] Socket exception: {e}")
        finally:
            with serial_lock:
                if ser and ser.is_open:
                    ser.write(b"0.000 0.000\n")

    server_sock.close()
    if ser:
        ser.close()
    print("[INFO] TCP Bridge shut down cleanly.")


if __name__ == "__main__":
    main()
