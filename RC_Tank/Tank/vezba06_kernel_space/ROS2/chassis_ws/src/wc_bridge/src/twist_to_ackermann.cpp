#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class TwistToAckermann : public rclcpp::Node
{
public:
  TwistToAckermann()
  : Node("twist_to_ackermann")
  {
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&TwistToAckermann::twist_cb, this, std::placeholders::_1)
    );

    pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/cmd", 10
    );

    this->declare_parameter("speed_scale", 1.0);
    this->declare_parameter("steering_scale", 1.0);
  }

private:
  void twist_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto out = ackermann_msgs::msg::AckermannDriveStamped();
    out.header.stamp = this->now();
    out.header.frame_id = "chassis";

    double speed_scale = this->get_parameter("speed_scale").as_double();
    double steer_scale = this->get_parameter("steering_scale").as_double();

    out.drive.speed = msg->linear.x * speed_scale;
    out.drive.steering_angle = msg->angular.z * steer_scale;

    pub_->publish(out);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToAckermann>());
  rclcpp::shutdown();
  return 0;
}

