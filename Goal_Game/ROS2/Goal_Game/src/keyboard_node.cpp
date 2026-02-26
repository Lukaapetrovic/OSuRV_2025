#include <ros/ros.h>
 #include <std_msgs/Int32.h>
 #include <termios.h>
 #include <unistd.h>
 #include <fcntl.h>
 #include <algorithm>
 
 int kbhit()
 {
 struct termios oldt, newt;
 int ch;
 int oldf;
 
 tcgetattr(STDIN_FILENO, &oldt);
 newt = oldt;
 newt.c_lflag &= ~(ICANON | ECHO);
 tcsetattr(STDIN_FILENO, TCSANOW, &newt);
 oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
 fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
 ch = getchar();
 
 tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
 fcntl(STDIN_FILENO, F_SETFL, oldf);
 
 if(ch != EOF)
 {
 ungetc(ch, stdin);
 return 1;
 }
 
 return 0;
 }
 
 char getKey()
 {
 char c = 0;
 if (kbhit())
 c = getchar();
 return c;
 }
 
 int main(int argc, char** argv)
 {
 ros::init(argc, argv, "keyboard_node");
 ros::NodeHandle nh;
 
 ros::Publisher pub = nh.advertise<std_msgs::Int32>("/servo_angle", 10);
 
 const int START_ANGLE = 120;
 const int MOVE_ANGLE = 59;
 const int MIN_ANGLE = 0;
 const int MAX_ANGLE = 180;
 
 int current_angle = START_ANGLE;
 int target_angle = START_ANGLE;
 int hold_cycles = 0; // broj ciklusa koliko drzimo ugao
 const int HOLD_MAX = 3; // 3 ciklusa = ~150 ms pri 20Hz
 
 ros::Rate rate(20); // 20 Hz
 
 ROS_INFO("Drzi 'a' levo, 'd' desno. Pusti za povratak. 'q' izlaz");
 
 while (ros::ok())
 {
 char key = getKey();
 
 if (key == 'a')
 {
 target_angle = std::min(START_ANGLE + MOVE_ANGLE, MAX_ANGLE);
 hold_cycles = HOLD_MAX;
 }
 else if (key == 'd')
 {
 target_angle = std::max(START_ANGLE - MOVE_ANGLE, MIN_ANGLE);
 hold_cycles = HOLD_MAX;
 }
 else if (key == 'q')
 {
 break;
 }
 
 // Ako nema tastera i jos nismo izdrzali dovoljno ciklusa, drzimo
 if (hold_cycles > 0)
 {
 hold_cycles--;
 current_angle = target_angle;
 }
 else
 {
 current_angle = START_ANGLE;
 }
 
 std_msgs::Int32 msg;
 msg.data = current_angle;
 pub.publish(msg);
 
 rate.sleep();
 }
 
 return 0;
 }

