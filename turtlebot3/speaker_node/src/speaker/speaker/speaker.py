import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.subscription = None  # Initially set to None
        self.timer = self.create_timer(3.0, self.timer_callback)
        
    def timer_callback(self):
        if self.subscription is None:
            self.subscription = self.create_subscription(
                String,
                '/label',
                self.listener_callback,
                10)
            self.get_logger().info('Subscribing...')
        else:
            self.destroy_subscription(self.subscription)
            self.subscription = None
            self.get_logger().info('Unsubscribing...')

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data == "person":
            subprocess.run(["aplay", "-D", "plughw:1,0", "human_detect.wav"])

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)

    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()