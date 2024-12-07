import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sshkeyboard import listen_keyboard, stop_listening
from color_led_interface.msg import LedCommand


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__("keyboard_publisher")
        self.publisher_ = self.create_publisher(LedCommand, "arduino_in", 10)

        # State tracking
        self.current_channel = None
        self.current_value = ""
        self.rgb_values = {"r": 0, "g": 0, "b": 0}

        self.get_logger().info(
            "Keyboard publisher started.\n"
            "- Press r/g/b to select channel, type value (0-255), enter to send\n"
            "- Press 'o' for full white, 'p' for off\n"
            "- ESC to exit"
        )

    def on_press(self, key):
        if key == "o":  # Set all channels to 255 (white)
            self.rgb_values = {"r": 255, "g": 255, "b": 255}
            self._publish_current_values()

        elif key == "p":  # Set all channels to 0 (off)
            self.rgb_values = {"r": 0, "g": 0, "b": 0}
            self._publish_current_values()

        elif key in ["r", "g", "b"]:
            self.current_channel = key
            self.current_value = ""
            self.get_logger().info(f"Selected {key.upper()} channel")

        elif key.isdigit() and self.current_channel:
            self.current_value += key
            self.get_logger().info(f"Current value: {self.current_value}")

        elif key == "enter" and self.current_channel and self.current_value:
            try:
                value = min(255, max(0, int(self.current_value)))
                self.rgb_values[self.current_channel] = value
                self._publish_current_values()
                self.current_channel = None
                self.current_value = ""
            except ValueError:
                self.get_logger().error("Invalid input value")
                self.current_value = ""

    def _publish_current_values(self):
        msg = LedCommand()
        msg.led_id = 1
        msg.r_value = self.rgb_values["r"]
        msg.g_value = self.rgb_values["g"]
        msg.b_value = self.rgb_values["b"]
        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published RGB values: R={msg.r_value}, G={msg.g_value}, B={msg.b_value}"
        )

    def start_listening(self):
        listen_keyboard(on_press=self.on_press, until="esc")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    try:
        node.create_timer(0.1, lambda: node.start_listening())
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down KeyboardPublisher node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
