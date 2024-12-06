import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sshkeyboard import listen_keyboard, stop_listening


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__("keyboard_publisher")

        # Create publisher
        self.publisher_ = self.create_publisher(String, "arduino_in", 10)

        # Command mappings
        self.key_commands = {
            "q": "LED1_ON",
            "w": "LED1_OFF",
            "a": "LED2_ON",
            "s": "LED2_OFF",
        }

        self.get_logger().info(
            "Keyboard publisher started. Press q/w for LED1, a/s for LED2. ESC to exit"
        )

    def on_press(self, key):
        if key in self.key_commands:
            msg = String()
            msg.data = self.key_commands[key]
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")

    def start_listening(self):
        listen_keyboard(on_press=self.on_press, until="esc")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    try:
        # Start keyboard listener in a way that doesn't block rclpy.spin
        node.create_timer(0.1, lambda: node.start_listening())
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down KeyboardPublisher node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
