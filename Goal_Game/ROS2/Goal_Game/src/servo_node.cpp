#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <wiringPi.h>
#include <softPwm.h>

#define SERVO_PIN 1   // GPIO18 = wiringPi pin 1

void servoCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int angle = msg->data;
    int pwm = 5 + angle * 20 / 180;  // map 0–180 → PWM

    softPwmWrite(SERVO_PIN, pwm);
    ROS_INFO("Servo angle: %d", angle);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "servo_node");
    ros::NodeHandle nh;

    wiringPiSetup();
    softPwmCreate(SERVO_PIN, 0, 200);

    ros::Subscriber sub = nh.subscribe("/servo_angle", 10, servoCallback);

    ros::spin();
    return 0;
}
