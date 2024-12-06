import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sshkeyboard import listen_keyboard, stop_listening

from color_led_interface.msg import LedCommand


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__("keyboard_publisher")

        # Create publisher
        self.publisher_ = self.create_publisher(LedCommand, "arduino_in", 10)

        # Command mappings
        self.key_commands = {
            "q": ("LED1_ON", 1, 255, 0, 0),
            "w": ("LED1_OFF", 1, 0, 0, 0),
            "a": ("LED2_ON", 2, 0, 255, 0),
            "s": ("LED2_OFF", 2, 0, 0, 0),
            "o": ("RGB_LED_ON", 1, 255, 255, 255),
            "p": ("RGB_LED_OFF", 1, 0, 0, 0),
        }

        self.get_logger().info(
            "Keyboard publisher started. Press q/w for LED1, a/s for LED2, o/p for RGB LED. ESC to exit"
        )

    def on_press(self, key):
        if key in self.key_commands:
            command, led_id, r_value, g_value, b_value = self.key_commands[key]
            msg = LedCommand()
            msg.led_id = led_id
            msg.r_value = r_value
            msg.g_value = g_value
            msg.b_value = b_value
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: {command} with values {msg}")

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
