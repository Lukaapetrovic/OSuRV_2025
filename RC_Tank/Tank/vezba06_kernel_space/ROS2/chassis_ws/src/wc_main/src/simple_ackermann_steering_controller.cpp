#include "simple_ackermann_steering_controller.hpp"

#include <cmath>
#include <iostream>

SimpleAckermannSteeringController::SimpleAckermannSteeringController()
: Node("simple_ackermann_steering_controller"),
  uart(nullptr),
  motors_en(true),
  all_motors_sim(false)
{
    // ---- parameters ----
    this->declare_parameter<bool>("all_motors_sim", false);
    this->get_parameter("all_motors_sim", all_motors_sim);

    RCLCPP_INFO(this->get_logger(),
                "Ackermann controller started (all_motors_sim = %s)",
                all_motors_sim ? "true" : "false");

    // ---- UART init ----
    if (!all_motors_sim)
    {
        uart = new SerialCPP::SerialCPP(
                                      "/dev/ttyAMA0",
                                      SerialCPP::BaudRate::BR_115200
                                                                    );

        if (!uart -> open())  {
            RCLCPP_FATAL(this->get_logger(), "Cannot open UART");
        }                      
    }

    // ---- subscribers ----
    motors_en_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "motors_en",
        10,
        std::bind(&SimpleAckermannSteeringController::motors_en_cb,
                  this,
                  std::placeholders::_1)
    );

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    cmd_sub_ = this->create_subscription<
    ackermann_msgs::msg::AckermannDriveStamped>(
        "cmd",
        qos,
        std::bind(&SimpleAckermannSteeringController::cmd_cb,
                  this,
                  std::placeholders::_1)
);

}

// -------------------------------------------------------------

void SimpleAckermannSteeringController::motors_en_cb(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    motors_en = msg->data;

    if (!motors_en)
    {
        RCLCPP_WARN(this->get_logger(), "Motors DISABLED");

        if (uart)
        {
            uart->writeBytes({0});
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Motors ENABLED");

        if (uart && !data.empty())
        {
            uart->writeBytes(data);
        }
    }
}

// -------------------------------------------------------------

void SimpleAckermannSteeringController::cmd_cb(
    const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    RCLCPP_WARN(this->get_logger(), "cmd_cb ENTERED");

    if (!motors_en) {
        RCLCPP_WARN(this->get_logger(), "motors disabled, ignoring cmd");
        return;
    }

    RCLCPP_WARN(this->get_logger(),
      "ACTIVE CMD: speed=%.2f steer=%.2f",
      msg->drive.speed,
      msg->drive.steering_angle
    );


    float speed = msg->drive.speed;
    float steering_angle = msg->drive.steering_angle;
    
    RCLCPP_INFO(this->get_logger(),
                 "cmd: speed=%.2f angle=%.2f",
                 msg->drive.speed,
                 msg->drive.steering_angle
                 );

    // limit reverse speed
    if (speed < -66.0f)
        speed = -66.0f;

    t_move direction = CM_IDLE;
    t_turn turn = CT_STRAIGHT;
    t_speed s_speed = S_INVALID;
    t_hardness hardness = CTH_FLOAT;

    // ---- direction ----
    if (speed > 0.1f)
        direction = CM_FORWARD;
    else if (speed < -0.1f)
        direction = CM_REVERSE;
    else
        direction = CM_IDLE;
        
    if (direction != CM_IDLE){
        hardness = CTH_BRAKE;
    }

    // ---- speed ----
    float abs_speed = std::fabs(speed);

    if (abs_speed <= 33.0f)
        s_speed = S_SLOW;
    else if (abs_speed <= 66.0f)
        s_speed = S_MEDIUM;
    else
        s_speed = S_FAST;

    // ---- turning ----
    if (steering_angle > 0.15f)
        turn = CT_LEFT;
    else if (steering_angle < -0.15f)
        turn = CT_RIGHT;
    else
        turn = CT_STRAIGHT;

    // ---- pack command ----
    uint8_t command = 0;
    command |= (static_cast<uint8_t>(s_speed) << 6);
    command |= (static_cast<uint8_t>(hardness) << 4);
    command |= (static_cast<uint8_t>(turn) << 2);
    command |= static_cast<uint8_t>(direction);
    

    //RCLCPP_INFO(this->get_logger(), "UART TX byte: 0x%02X", command);
    data.clear();
    data.push_back(command);



    // ---- send ----
    if (uart && motors_en)
    {
        uart->writeBytes(data);
    }

    
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SimpleAckermannSteeringController>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

