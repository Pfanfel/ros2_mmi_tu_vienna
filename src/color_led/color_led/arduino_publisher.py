import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class ArduinoPublisher(Node):
    def __init__(self):
        super().__init__("arduino_publisher")

        # Parameters
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 9600)
        self.declare_parameter("publish_topic", "arduino_out")

        # Serial connection
        serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)

        # Publisher
        publish_topic = (
            self.get_parameter("publish_topic").get_parameter_value().string_value
        )
        self.publisher_ = self.create_publisher(String, publish_topic, 10)

        # Timer to read from Arduino
        self.timer = self.create_timer(0.1, self.read_from_arduino)

    def read_from_arduino(self):
        if self.serial_conn.in_waiting > 0:
            try:
                data = self.serial_conn.readline().decode("utf-8").strip()
                if data:
                    msg = String()
                    msg.data = data
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published: {msg.data}")
            except Exception as e:
                self.get_logger().error(f"Error reading from serial: {e}")

    def destroy_node(self):
        self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ArduinoPublisher node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
