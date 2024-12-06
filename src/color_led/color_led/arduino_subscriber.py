import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class ArduinoSubscriber(Node):
    def __init__(self):
        super().__init__("arduino_subscriber")

        # Parameters
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 9600)
        self.declare_parameter("subscribe_topic", "arduino_in")

        # Serial connection
        serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)

        # Subscriber
        subscribe_topic = (
            self.get_parameter("subscribe_topic").get_parameter_value().string_value
        )
        self.subscription_ = self.create_subscription(
            String, subscribe_topic, self.send_to_arduino, 10
        )

    def send_to_arduino(self, msg):
        try:
            self.serial_conn.write((msg.data + "\n").encode("utf-8"))
            self.get_logger().info(f"Sent to Arduino: {msg.data}")
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
