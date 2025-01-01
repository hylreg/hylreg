#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class PointCloudSub : public rclcpp::Node{

    public:
    PointCloudSub() : Node("point_cloud_sub"){
         sub = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&PointCloudSub::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const{
        printf("I heard: [%s]\n", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;

};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world my_point_cloud package PointCloudSub\n");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSub>());
    rclcpp::shutdown();


  return 0;
}
