#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"


class SensorDataSubscriber : public rclcpp::Node
{
public:
  SensorDataSubscriber()
  : Node("sensor_data_subscriber")
  {
    rclcpp::QoS qos_profile(10);
  qos_profile.best_effort();
    // 创建图像订阅者
    rgb_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/rgb/image_raw", qos_profile, std::bind(&SensorDataSubscriber::rgb_callback, this, std::placeholders::_1));

    rgbd_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/rgbd/depth/image_raw", qos_profile, std::bind(&SensorDataSubscriber::rgbd_callback, this, std::placeholders::_1));

    // // 创建点云订阅者
    // point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //   "/camera/depth/image_raw", 10, std::bind(&SensorDataSubscriber::point_cloud_callback, this, std::placeholders::_1));
  }

private:
  // 图像回调
  void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // 将 ROS2 图像消息转换为 OpenCV 图像
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      // 处理图像，例如显示图像
      cv::imshow("Received rgb", cv_ptr->image);
      cv::waitKey(1);  // 需要调用等待函数来显示图像
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
    }
  }


  void rgbd_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
        // RCLCPP_INFO(this->get_logger(), "Received Image: width=%d, height=%d, encoding=%s, step=%d",
        //             msg->width, msg->height, msg->encoding.c_str(), msg->step);
      // 将 ROS2 图像消息转换为 OpenCV 图像
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

      // 处理图像，例如显示图像
      cv::imshow("Received rgbd", cv_ptr->image);
      cv::waitKey(1);  // 需要调用等待函数来显示图像
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
    }
  }

  // 图像订阅者
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgbd_subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDataSubscriber>());
  rclcpp::shutdown();
  return 0;
}
