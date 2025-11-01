import cv2
import threading
import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class CameraSubscriber(Node):

    def __init__(self):

        super().__init__('cam_sub')

        self.cam_sub = self.create_subscription(Image, '/camera/raw_image', self.camera_CB, 10)
        self.cam_sub

    def camera_CB(self, data):
        global img #NOTE: Define global variable (img) so that other nodes can use it 
        img = bridge.imgmsg_to_cv2(data, "bgr8")

class YoloSubscriber(Node):

    def __init__(self):

        super().__init__('yolo_sub')

        self.yolo_sub = self.create_subscription(Yolov8Inference, "/yolov8_inference", self.yolo_CB, 10)
        self.yolo_sub

        self.img_pub = self.create_publisher(Image, "inference_result_cv2", 1)

    def yolo_CB(self, data):
        global img
        
        for r in data.yolov8_inference:
            class_name = r.class_name
            top = r.top
            left = r.left
            bottom = r.bottom
            right = r.right
            
            #YoloSubscriber.get_logger().info(f"{class_name} : {top}, {left}, {bottom}, {right}")
            cv2.rectangle(img, (top, left), (bottom, right), (255,255,0))

        self.cnt = 0
        img_msg = bridge.cv2_to_imgmsg(img)
        self.img_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    yolo_sub = YoloSubscriber()
    cam_sub = CameraSubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(yolo_sub)
    executor.add_node(cam_sub)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = yolo_sub.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt():
        pass
    rclpy.shutdown()
    executor_thread.join()

if __name__ == "__main__":
    main()
