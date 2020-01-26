#ifndef CMD_VEL_H_
#define CMD_VEL_H_

///LeftMotor Pins
#define INA_1 12
#define INB_1 13
#define PWM_1 PC_6
///Right Motor Pins
#define INA_2 5
#define INB_2 6
#define PWM_2 PC_5

// min and max vel
#define MAX_LINEAR_VEL 1.15
#define MIN_LINEAR_VEL -1.15


// include libraries
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int64.h>

// global var(s)
int left_PWM = 0;
int right_PWM = 0;


// callback functions have to be declared before their subscribers to compile
void cmd_cb(const geometry_msgs::Twist& msg);

// nodehandle, publishers, subscribers
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &cmd_cb);
std_msgs::String pub_string;
ros::Publisher status_pub("cmd_vel_debug_topic", &pub_string);
std_msgs::Int64 pwm_pub;
ros::Publisher pwm_status("motor_pwm", &pwm_pub);

// other functions
void stop_moving();
void right_motor(int pwm);
void left_motor(int pwm);

#endif
