import rclpy
from rclpy.node import Node
from color_led_interface.msg import LedCommand
import serial


class ArduinoSubscriber(Node):
    def __init__(self):
        super().__init__("arduino_subscriber")

        # Create subscriber
        self.subscription = self.create_subscription(
            LedCommand, "arduino_in", self.send_to_arduino, 10
        )
        self.subscription  # prevent unused variable warning

        # Initialize serial connection
        self.serial_conn = serial.Serial("/dev/ttyACM0", 9600)

    def send_to_arduino(self, msg):
        try:
            command = f"{msg.led_id},{msg.r_value},{msg.g_value},{msg.b_value}\n"
            self.serial_conn.write(command.encode("utf-8"))
            self.get_logger().info(f"Sent to Arduino: {command}")
        except Exception as e:
            self.get_logger().error(f"Error writing to serial: {e}")

    def destroy_node(self):
        self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ArduinoSubscriber node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
