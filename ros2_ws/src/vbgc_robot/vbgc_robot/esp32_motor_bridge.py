"""
===============================================================================
Project: Vision-Based Gesture-Controlled Person-Following Robot
Package: vbgc_robot
File: ros2_ws/src/vbgc_robot/vbgc_robot/esp32_motor_bridge.py
Description: Translates /cmd_vel into UART ASCII packets for the ESP32.
             Includes auto-reconnect logic, dialout permission hints, and
             failsafe zero-velocity heartbeats.
===============================================================================
"""

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ESP32MotorBridge(Node):

    def __init__(self):
        super().__init__("esp32_motor_bridge")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("command_timeout", 0.5)

        self.port = self.get_parameter("serial_port").get_parameter_value().string_value
        self.baud = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.timeout = self.get_parameter("command_timeout").get_parameter_value().double_value

        self.serial_conn = None
        self.last_cmd_time = self.get_clock().now()

        self.connect_serial()

        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)
        self.timer = self.create_timer(0.1, self.watchdog_check)
        self.get_logger().info("ESP32 Motor Bridge Node initialized.")

    def connect_serial(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Connected to ESP32 on {self.port} at {self.baud} baud.")
        except serial.SerialException as e:
            self.serial_conn = None
            self.get_logger().error(f"Cannot open {self.port}: {e}. Check USB cable and dialout group!")

    def send_command(self, v_linear, w_angular):
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        packet = f"{v_linear:.3f} {w_angular:.3f}\n"
        try:
            self.serial_conn.write(packet.encode("utf-8"))
        except serial.SerialException as err:
            self.get_logger().warn(f"UART Write failed: {err}")
            self.serial_conn = None

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self.send_command(msg.linear.x, msg.angular.z)

    def watchdog_check(self):
        if self.serial_conn is None:
            self.connect_serial()
            return

        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.timeout:
            self.send_command(0.0, 0.0)

    def destroy_node(self):
        if self.serial_conn:
            try:
                self.send_command(0.0, 0.0)
                self.serial_conn.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
