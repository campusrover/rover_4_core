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

// Encoder defines
#define Left_Encoder_PinA 31
#define Left_Encoder_PinB 32
#define Right_Encoder_PinA 33
#define Right_Encoder_PinB 34
// set max and min encoder values to be 2^32. this could be higher
#define ENCODER_MAX 4294967296
#define ENCODER_MIN -4294967296

// Sonar defines
#define Trig 10
#define Echo 9


// include libraries
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Header.h>

// global var(s)
// motor PWM's
int left_PWM = 0;
int right_PWM = 0;
// Encoder ticks
volatile long Left_Encoder_Ticks = 0;
//Variable to read current state of left encoder pin
volatile bool LeftEncoderBSet;
volatile long Right_Encoder_Ticks = 0;
//Variable to read current state of right encoder pin
volatile bool RightEncoderBSet;
// Sonar 
long duration, cm;


// callback functions have to be declared before their subscribers to compile
void cmd_cb(const geometry_msgs::Twist& msg);

// nodehandle, publishers, subscribers
ros::NodeHandle nh;
// subscribers
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &cmd_cb);
// publishers (and the things they publish)
std_msgs::String pub_string;
ros::Publisher status_pub("cmd_vel_debug_topic", &pub_string);
std_msgs::Int64 pwm_pub;
ros::Publisher pwm_status("motor_pwm", &pwm_pub);
std_msgs::Int64 enc_l;
ros::Publisher left_enc_pub("encoder_left", &enc_l);
std_msgs::Int64 enc_r;
ros::Publisher right_enc_pub("encoder_right", &enc_r);
sensor_msgs::Range sonar;
ros::Publisher sonar_pub("sonar_front", &sonar);
std_msgs::Header sonar_head;

// other functions
void right_motor(int pwm);
void left_motor(int pwm);
void do_right_encoder();
void do_left_encoder();
void Update_Ultra_Sonic();

#endif
