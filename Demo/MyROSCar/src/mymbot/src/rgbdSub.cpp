#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>


class SensorDataSubscriber : public rclcpp::Node
{
public:
  SensorDataSubscriber()
  : Node("sensor_data_subscriber")
  {
    rclcpp::QoS qos_profile(10);
  qos_profile.best_effort();
    // 创建图像订阅者
    // rgb_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    //   "/rgb/image_raw", qos_profile, std::bind(&SensorDataSubscriber::rgb_callback, this, std::placeholders::_1));

    // rgbd_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    //   "/rgbd/depth/image_raw", qos_profile, std::bind(&SensorDataSubscriber::rgbd_callback, this, std::placeholders::_1));

    rgb_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/color/image_raw", qos_profile, std::bind(&SensorDataSubscriber::rgb_callback, this, std::placeholders::_1));

    rgbd_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/depth/image_raw", qos_profile, std::bind(&SensorDataSubscriber::rgbd_callback, this, std::placeholders::_1));

    // 创建点云订阅者
    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/depth/points", qos_profile, std::bind(&SensorDataSubscriber::point_cloud_callback, this, std::placeholders::_1));
  
  // point_cloud_color_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //     "/camera/depth/color/points", qos_profile, std::bind(&SensorDataSubscriber::point_cloud_color_callback, this, std::placeholders::_1));


    // viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("viewer");
    // viewer_->setBackgroundColor(0, 0, 0);  // 设置背景颜色为黑色
    // viewer_->initCameraParameters();  // 初始化相机参数
  
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
      cv::imshow("Received rgbd", cv_ptr->image);
      cv::waitKey(1);  // 需要调用等待函数来显示图像
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
    }
  }

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    try {

        // 将 PointCloud2 转换为 PCL 格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        RCLCPP_INFO(this->get_logger(), "Received PointCloud2 with width: %d, height: %d", msg->width, msg->height);


    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Could not process pointCloud: %s", e.what());
    }
  }

  //     void point_cloud_color_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  // {
  //   try {

  //       // 将 PointCloud2 转换为 PCL 格式
  //       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //       pcl::fromROSMsg(*msg, *cloud);
  //       RCLCPP_INFO(this->get_logger(), "Received PointCloud2 with width: %d, height: %d", msg->width, msg->height);

  //       // // 在 PCLVisualizer 中添加或更新点云
  //       // if (!viewer_->updatePointCloud(cloud, "cloud")) {
  //       //     viewer_->addPointCloud(cloud, "cloud");
  //       // }

  //       // // 持续更新可视化
  //       // viewer_->spinOnce(100);

  //   } catch (cv_bridge::Exception &e) {
  //     RCLCPP_ERROR(this->get_logger(), "Could not process pointCloud: %s", e.what());
  //   }
  // }

    //     // 定义一个可视化窗口，确保它只初始化一次
    // void spin()
    // {
    //     // 进入循环，持续运行可视化
    //     while (rclcpp::ok()) {
    //         rclcpp::spin_some(this->get_node_base_interface());  // 调用 ROS2 spin 来处理回调
    //         viewer_->spinOnce(100);  // 更新可视化
    //     }
    // }

  // 图像订阅者
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgbd_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_color_subscription_;

  // pcl::visualization::PCLVisualizer::Ptr viewer_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDataSubscriber>());
  rclcpp::shutdown();
  return 0;
}
