#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>

#ifndef TIVAC_H_
#define TIVAC_H_

///LeftMotor Pins
#define INA_1 5
#define INB_1 6
#define PWM_1 PC_5
///Right Motor Pins
#define INA_2 12
#define INB_2 13
#define PWM_2 PC_6

// min and max vel
#define MAX_LINEAR_VEL 1.15
#define MIN_LINEAR_VEL -1.15
// ramp values - basically saying that if the difference in PWM's over time is grater than RAMP_THRESHOLD, then PWM will increment by RAMP_FACTOR
#define RAMP_FACTOR 10
#define RAMP_THRESHOLD 15

// Encoder defines
#define Left_Encoder_PinA 33
#define Left_Encoder_PinB 34
#define Right_Encoder_PinA 31
#define Right_Encoder_PinB 32
// set max and min encoder values to be 2^32. this could be higher
#define ENCODER_MAX 4294967296
#define ENCODER_MIN -4294967296

// motor PWM's
// Target PWM set based on twist
int left_PWM = 0;
int right_PWM = 0;
// PWM that is actuallys ent to motor (by ramp)
int left_PWM_out = 0;
int right_PWM_out = 0;
// previous PWM (from ramp)
int prev_left_PWM = 0;
int prev_right_PWM = 0;
// Encoder ticks
volatile long Left_Encoder_Ticks = 0;
//Variable to read current state of left encoder pin
volatile bool LeftEncoderBSet;
volatile long Right_Encoder_Ticks = 0;
//Variable to read current state of right encoder pin
volatile bool RightEncoderBSet;

// callback functions have to be declared before their subscribers to compile
void cmd_cb(const geometry_msgs::Twist& msg);
void left_vel_cb(const std_msgs::Float32& msg);
void right_vel_cb(const std_msgs::Float32& msg);

// subscribers
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &cmd_cb);
ros::Subscriber<std_msgs::Float32> left_vel_sub("left_wheel_vel", &left_vel_cb);
ros::Subscriber<std_msgs::Float32> right_vel_sub("right_wheel_vel", &right_vel_cb);
// publishers (and the things they publish)
std_msgs::String pub_string;
ros::Publisher status_pub("cmd_vel_debug_topic", &pub_string);
std_msgs::Int64 pwm_pub;
ros::Publisher pwm_status("motor_pwm", &pwm_pub);
std_msgs::Int64 enc_l;
ros::Publisher left_enc_pub("encoder_left", &enc_l);
std_msgs::Int64 enc_r;
ros::Publisher right_enc_pub("encoder_right", &enc_r);

// other functions
void right_motor(int pwm);
void left_motor(int pwm);
void do_right_encoder();
void do_left_encoder();
void updateMotors();

#endif
