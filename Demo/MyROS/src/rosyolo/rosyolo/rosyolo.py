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
            # 处理接收到的点云数据
            # self.get_logger().info('Received point cloud data')

            # 示例：转换 PointCloud2 数据为点列表
            points = pc2.read_points(msg, field_names=("x", "y", "z","rgb"), skip_nans=True)
            processed_points = []

            # 这里做一个简单的处理，例如只取 x, y, z 坐标中的某些值
            for p in points:
                x, y, z, rgb= p
                # 你可以在这里加入一些处理逻辑，比如滤波或转换
                modified_rgb = self.modify_rgb(rgb)
                processed_points.append([x, y, z, modified_rgb])

                self.get_logger().info(f"Modified RGB: {modified_rgb:#08x}")

            # 创建一个新的 PointCloud2 消息，发布处理后的点云
            header = std_msgs.msg.Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = msg.header.frame_id

            # 在这里，我们将RGB信息添加回点云数据
            processed_cloud = self.add_rgb_to_point_cloud(header, processed_points)

            # 发布处理后的点云数据
            self.depth_image_pub.publish(processed_cloud)
            # self.get_logger().info('Processed point cloud data published.')
    def modify_rgb(self, rgb):


        r = 127
        g = 0
        b = 0
        # rgb_int = int(rgb)

        # # 提取 RGB 分量
        # r = (rgb_int >> 16) & 0xFF  # 红色分量
        # g = (rgb_int >> 8) & 0xFF   # 绿色分量
        # b = rgb_int & 0xFF          # 蓝色分量


    
        # 增加亮度（10%）
        r = min(255, int(r * 1.1))
        g = min(255, int(g * 1.1))
        b = min(255, int(b * 1.1))

        # 重新组合为一个 RGB 整数值
        return (r << 16) | (g << 8) | b

    def add_rgb_to_point_cloud(self, header, points):
        # 将处理过的 RGB 信息加回到 PointCloud2 数据
        new_data = []
        for i, point in enumerate(points):
            x, y, z, rgb = point
            # 将 RGB 通道打包回一个 uint32
            packed_rgb = struct.unpack('I', struct.pack('BBBB', (rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF, 255))[0]

            new_data.append([x, y, z, packed_rgb])

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]

        return pc2.create_cloud(header,fields, new_data)






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
