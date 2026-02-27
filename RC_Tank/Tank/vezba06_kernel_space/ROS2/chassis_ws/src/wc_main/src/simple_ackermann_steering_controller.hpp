#ifndef SIMPLE_ACKERMANN_STEERING_CONTROLLER_HPP
#define SIMPLE_ACKERMANN_STEERING_CONTROLLER_HPP

#include <vector>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include "SerialCPP.h"

// movement
enum t_move
{
    CM_IDLE = 0,
    CM_FULL_BRAKE = 1,
    CM_FORWARD = 2,
    CM_REVERSE = 3
};

// turning
enum t_turn
{
    CT_STRAIGHT = 0,
    CT_RIGHT = 1,
    CT_LEFT = 2,
    CT_INVALID = 3
};

//hardness
enum t_hardness
{
  CTH_FLOAT = 0,
  CTH_BRAKE = 1,
  CTH_IN_PLACE = 2,
  CTH_INVALID = 3
};

// speed
enum t_speed
{
    S_SLOW = 0,
    S_MEDIUM = 1,
    S_FAST = 2,
    S_INVALID = 3
};

class SimpleAckermannSteeringController : public rclcpp::Node
{
public:
    SimpleAckermannSteeringController();

private:
    void motors_en_cb(const std_msgs::msg::Bool::SharedPtr msg);
    void cmd_cb(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

    // UART
    SerialCPP::SerialCPP *uart;

    // last command
    std::vector<uint8_t> data;

    bool motors_en;
    bool all_motors_sim;

    // ROS2 subscriptions
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motors_en_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_sub_;
};

#endif
