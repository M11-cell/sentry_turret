import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_pub_')
        self.cam_pub_ = self.create_publisher(Image, '/camera/raw_image', 10)
        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            self.cam_pub_.publish(self.br.cv2_to_imgmsg(frame))
        self.get_logger().info('publishing video frame')





def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisherNode()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
