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

//// Sonar defines
//#define Trig 10
//#define Echo 9


// include libraries
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#include <ros.h>
#include <math.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

// global var(s)
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
//// Sonar 
//long duration, cm;
// IMU
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
// battery
float battery_level = 12; // why not? this is the default value from the book


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
//sensor_msgs::Range sonar;
//ros::Publisher sonar_pub("sonar_front", &sonar);
//std_msgs::Header sonar_head;
sensor_msgs::Imu imu;
ros::Publisher imu_pub("imu", &imu);

// other functions
void right_motor(int pwm);
void left_motor(int pwm);
void do_right_encoder();
void do_left_encoder();
void updateMotors();
//void Update_Ultra_Sonic();
//void updateSonar();
void updateIMU();

#endif
