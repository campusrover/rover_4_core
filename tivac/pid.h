#include <ros.h>
#include <std_msgs/Float32.h>
#include "motor.h"
#include "tivac.h"


// PID defines
/*
#define PROPORTIONAL_GAIN 0
#define INTEGRAL_GAIN 0
#define DERIVITIVE_GAIN 0
*/
double left_accumulated_error = 0;
double right_accumulated_error = 0;
double left_derivitive_error = 0;
double right_derivitive_error = 0;
double left_previous_error = 0;
double right_previous_error = 0;
// PID Debugging
double PROPORTIONAL_GAIN = 0;
double INTEGRAL_GAIN = 0;
double DERIVITIVE_GAIN = 0;


void p_cb(const std_msgs::Float32& msg);
void i_cb(const std_msgs::Float32& msg);
void d_cb(const std_msgs::Float32& msg);



ros::Subscriber<std_msgs::Float32> p_gain_sub("p_gain", &p_cb);
ros::Subscriber<std_msgs::Float32> i_gain_sub("i_gain", &i_cb);
ros::Subscriber<std_msgs::Float32> d_gain_sub("d_gain", &d_cb);


void full_pid()  {
    // saved from tivac.ino, while moving away from full PID
    // get the change in encoders, calculate pervious speed
  int left_encoder_change = encoder_difference(Left_Encoder_Ticks, prev_left_encoder_ticks);
  int right_encoder_change = encoder_difference(Right_Encoder_Ticks, prev_right_encoder_ticks);
  time_now = nh.now().toSec();
  double elapsed_time = time_now - time_last;
  double wheel_circumference = 2 * WHEEL_RADIUS * M_PI;
  double left_vel_actual = left_encoder_change / elapsed_time * wheel_circumference / ENCODER_TICKS_PER_REV;  // ticks / second to m/s
  double right_vel_actual = right_encoder_change / elapsed_time * wheel_circumference / ENCODER_TICKS_PER_REV;
  double left_error =  left_vel - left_vel_actual;  // positive error = too slow (increase speed), negative error = too fast (decrease speed)
  double right_error = right_vel - right_vel_actual;
  // P roprotional
  // no action needed
  // I ntegral
  left_accumulated_error += left_error * elapsed_time;
  right_accumulated_error += right_error * elapsed_time;
  // D erivative 
  left_derivitive_error = (left_error - left_previous_error) / elapsed_time;  // further change in error from 0, faster error is changing (whether it is getting bigger or smaller, cannot say alone)
  right_derivitive_error = (right_error - right_previous_error) / elapsed_time;
  left_previous_error = left_error;
  right_previous_error = right_error;

  double left_PID_sum = (left_error * PROPORTIONAL_GAIN) + (left_accumulated_error * INTEGRAL_GAIN) + (left_derivitive_error * DERIVITIVE_GAIN);  // PID sum is in arbitrary output units
  double right_PID_sum = (right_error * PROPORTIONAL_GAIN) + (right_accumulated_error * INTEGRAL_GAIN) + (right_derivitive_error * DERIVITIVE_GAIN);

  // convert PID output to a PWM
  // going off the assumption that the PID sum is some sort of total representation of error, then that sum can be converted to a PWM and added to the previous PWM (could be negative?)
  // let's also assume our top speed is 1 m/s and that every .1 m/s of error will result in a change in 25 PWM
  left_PWM_out += (int)(left_PID_sum / 0.1 * 25);
  right_PWM_out += (int)(right_PID_sum / 0.1 * 25);
  // push time back for next loop
  time_last = time_now;
}

void p_cb(const std_msgs::Float32& msg) {
  PROPORTIONAL_GAIN = msg.data;
}

void i_cb(const std_msgs::Float32& msg) {
  INTEGRAL_GAIN = msg.data;
}

void d_cb(const std_msgs::Float32& msg) {
  DERIVITIVE_GAIN = msg.data;
}