import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSubscriberNode(Node):
    def __init__(self):

        super().__init__('image_sub_')

        self.br = CvBridge()

        self.cam_sub_ = self.create_subscription(Image, '/camera/raw_image', self.listener_CB, 10)
        self.cam_sub_

    def listener_CB(self, data):
        self.get_logger().info('receiving video data')
        current_frame = self.br.imgmsg_to_cv2(data)
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    sub_node_ = ImageSubscriberNode()
    rclpy.spin(sub_node_)
    sub_node_.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()