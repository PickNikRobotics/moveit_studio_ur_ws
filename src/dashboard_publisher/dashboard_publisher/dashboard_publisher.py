import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PilImage, ImageDraw, ImageFont
import numpy as np

class DashboardPublisher(Node):
    def __init__(self):
        super().__init__('dashboard_publisher')

        # Define the QoS profile with SERVICES_DEFAULT settings
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)

        self.publisher_ = self.create_publisher(Image, 'dashboard_image', qos_profile)
        self.battery_percent = 100
        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.get_logger().info("Dashboard Publisher Node has started.")

    def timer_callback(self):
        # Create an image with PIL
        pil_image = PilImage.new('RGB', (320, 240), color=(255, 255, 255))
        draw = ImageDraw.Draw(pil_image)

        # Load a font (using a default font here)
        font = ImageFont.load_default()
        
        # Add text to the image
        # text = "Battery: "  low"
        text = f"Battery level: {self.battery_percent}%"
        text_position = (50, 100)
        text_color = (255, 0, 0)
        draw.text(text_position, text, fill=text_color, font=font)
        
        # Convert PIL image to a ROS Image message using CvBridge
        cv_image = np.array(pil_image)
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
        
        # Publish the message
        self.publisher_.publish(ros_image)
        self.get_logger().info(f"Published 1x an image with 'Battery {self.battery_percent}%'")
        # self.publisher_.publish(ros_image)
        # self.get_logger().info(f"Published 2x an image with 'Battery {self.battery_percent}%'")
        
        # Decrement battery life or reset to 100%
        self.battery_percent = self.battery_percent - 3 if self.battery_percent > 2 else 100

def main(args=None):
    rclpy.init(args=args)
    node = DashboardPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
