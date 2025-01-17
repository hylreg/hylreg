import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge

from ultralytics import YOLO
import std_msgs.msg

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
            # 处理接收到的点云数据
            self.get_logger().info('Received point cloud data')

            # 示例：转换 PointCloud2 数据为点列表
            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            processed_points = []

            # 这里做一个简单的处理，例如只取 x, y, z 坐标中的某些值
            for p in points:
                x, y, z = p
                # 你可以在这里加入一些处理逻辑，比如滤波或转换
                processed_points.append([x, y, z])

            # 创建一个新的 PointCloud2 消息，发布处理后的点云
            header = std_msgs.msg.Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = msg.header.frame_id
            
            # 你需要根据实际数据处理创建一个新的 PointCloud2 消息
            processed_cloud = pc2.create_cloud_xyz32(header, processed_points)

            # 发布处理后的点云数据
            self.depth_image_pub.publish(processed_cloud)
            self.get_logger().info('Processed point cloud data published.')






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
