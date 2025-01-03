#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/ros/conversions.h>

class SensorDataSubscriber : public rclcpp::Node
{
public:
  SensorDataSubscriber()
  : Node("sensor_data_subscriber")
  {
    rclcpp::QoS qos_profile(10);
  qos_profile.best_effort();
    // 创建图像订阅者
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", qos_profile, std::bind(&SensorDataSubscriber::image_callback, this, std::placeholders::_1));

    // // 创建点云订阅者
    // point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //   "/camera/depth/image_raw", 10, std::bind(&SensorDataSubscriber::point_cloud_callback, this, std::placeholders::_1));
  }

private:
  // 图像回调
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // 将 ROS2 图像消息转换为 OpenCV 图像
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

      // 处理图像，例如显示图像
      cv::imshow("Received Image", cv_ptr->image);
      cv::waitKey(1);  // 需要调用等待函数来显示图像
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
    }
  }

  // 点云回调
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    printf("Hello");
    RCLCPP_INFO(this->get_logger(), "Callback triggered!");
            // 将 PointCloud2 转换为 PCL 格式
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
    // 在此可以处理点云数据
    RCLCPP_INFO(this->get_logger(), "Received PointCloud2 with width: %d, height: %d", msg->width, msg->height);
    // 例如，可以使用 PCL 库进行点云处理
  }

  // 图像和点云订阅者
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDataSubscriber>());
  rclcpp::shutdown();
  return 0;
}
