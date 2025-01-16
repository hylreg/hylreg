import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

from ultralytics import YOLO

class YOLODetectionNode(Node):



    def __init__(self):
        super().__init__('ultralytics_node')
        self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_callback, 10)
        self.det_image_pub = self.create_publisher(Image, "/ultralytics/detection/image", 10)
        self.seg_image_pub = self.create_publisher(Image, "/ultralytics/segmentation/image", 10)
        self.detection_model = YOLO("src/rosyolo/resource/yolo11n.pt")
        self.segmentation_model = YOLO("src/rosyolo/resource/yolo11n-seg.pt")
        self.bridge = CvBridge()
        # self.yolo = YOLO()

    def image_callback(self, msg):


        if self.det_image_pub.get_subscription_count() > 0:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            det_result = self.detection_model(cv_image,device=0)
            det_annotated = det_result[0].plot(show=False)
            self.det_image_pub.publish(self.bridge.cv2_to_imgmsg(det_annotated))

        if self.seg_image_pub.get_subscription_count() > 0:
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            det_result = self.segmentation_model(cv_image,device=0)
            det_annotated = det_result[0].plot(show=False)
            self.seg_image_pub.publish(self.bridge.cv2_to_imgmsg(det_annotated))





def main(args=None):
    rclpy.init(args=args)

    # Create and spin the node
    node = YOLODetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()
