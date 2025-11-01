import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_pub_')

        self.cam = cv2.VideoCapture(0)
        self.br = CvBridge()

        timer_period = 0.1

        self.cam_pub_ = self.create_publisher(Image, '/camera/raw_image', 10)
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        self.i = 0 # This is a counter to trakc how many images are published
        

    def timer_callback(self):

        ret, frame = self.cam.read()

        frame = cv2.resize(frame, (820,640), interpolation=cv2.INTER_CUBIC)

        #if frames are legible
        if ret == True:
            img_msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
            self.cam_pub_.publish(img_msg)

        self.get_logger().info('publishing video frame')
        #updating image counter
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisherNode()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
