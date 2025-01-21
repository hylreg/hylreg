import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge
import numpy as np

from ultralytics import YOLO
import std_msgs.msg
import struct
from sensor_msgs.msg import PointField

class YOLODetectionNode(Node):



    def __init__(self):
        super().__init__('ultralytics_node')
        self.image_sub = self.create_subscription(Image, "/camera/color/image_raw", self.image_callback, 10)
        self.depth_sub = self.create_subscription(PointCloud2, "/camera/depth_registered/points", self.depth_callback, 10)

        self.det_image_pub = self.create_publisher(Image, "/ultralytics/detection/image", 10)
        self.seg_image_pub = self.create_publisher(Image, "/ultralytics/segmentation/image", 10)
        self.depth_image_pub = self.create_publisher(PointCloud2, "/ultralytics/segmentation/PointCloud2", 10)
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

    def depth_callback(self, msg):



        if self.depth_image_pub.get_subscription_count() > 0:
            points = pc2.read_points(msg, field_names=("x", "y", "z","rgb"), skip_nans=True)
            processed_points = []

            image_width = 640
            image_height = 480

            x_min, x_max = -image_width/2, image_width/2  # 示例范围
            y_min, y_max = -image_height/2, image_height/2  # 示例范围

            resolutionX = image_width / (x_max - x_min)
            resolutionY = image_height / (y_max - y_min)

            image = np.zeros((image_height, image_width, 3), dtype=np.uint8)


            for p in points:
                x, y, z, rgb= p
                processed_points.append([x, y, z])

                rgb = int(rgb)
                # r = 127
                r = (rgb >> 16) & 0x0000ff
                g = (rgb >> 8) & 0x0000ff
                b = rgb & 0x0000ff

                bgr = (b, g, r)



                # 映射到图像坐标
                x_img = int((x - x_min) * resolutionX)
                y_img = int((y - y_min) * resolutionY)

                # 确保坐标在图像范围内
                x_img = max(0, min(x_img, image_width - 1))
                y_img = max(0, min(y_img, image_height - 1))

                image[y_img, x_img] = bgr

                # image[abs(int(y)), abs(int(x))] = bgr
                # self.get_logger().info(f"X: {x}, Y: {y}, Z: {z}")


            # cv2.circle(image, (320, 240), 100, (0, 0, 255), -1)

            # image[:] = bgr
            cv2.imshow('depth', image)
            cv2.waitKey(1)




            header = std_msgs.msg.Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = msg.header.frame_id

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                # PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
            ]

            processed_cloud = pc2.create_cloud(header,fields, processed_points)
            self.depth_image_pub.publish(processed_cloud)



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
