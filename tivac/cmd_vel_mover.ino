
#include "cmd_vel.h"


void cmd_cb(const geometry_msgs::Twist& msg) {
  // twist to velocity based on https://github.com/PacktPublishing/Learning-Robotics-using-Python-Second-Edition/blob/master/chapter_8_code/chefbot_bringup/scripts/twist_to_motors.py
  // width of robot
  double width = 0.26; //meters
  // linear component
  float linear_vel = msg.linear.x;
  // angular component
  float angular_vel = msg.angular.z;
  // combine values 
  float left_vel = 1.0 * linear_vel - angular_vel * width / 2.0;
  float right_vel = 1.0 * linear_vel + angular_vel * width / 2.0;
  // constrain value (left)
  if (left_vel > MAX_LINEAR_VEL) {
    left_vel = MAX_LINEAR_VEL;
  } else if (left_vel < MIN_LINEAR_VEL) {
    left_vel = MIN_LINEAR_VEL;
  }
  // constrain value (right)
  if (right_vel > MAX_LINEAR_VEL) {
    right_vel = MAX_LINEAR_VEL;
  } else if (right_vel < MIN_LINEAR_VEL) {
    right_vel = MIN_LINEAR_VEL;
  }
  // convert to PWM (left)
  if (left_vel > 0) {
    left_PWM = (left_vel / MAX_LINEAR_VEL) * 255;
  } else if (left_vel < 0) {
    left_PWM = (left_vel / MIN_LINEAR_VEL) * 255;
  } else {
    left_PWM = 0;
  }
  // convert to PWM (right)
  if (right_vel > 0) {
    right_PWM = (right_vel / MAX_LINEAR_VEL) * 255;
  } else if (right_vel < 0) {
    right_PWM = (right_vel / MIN_LINEAR_VEL) * 255;
  } else {
    right_PWM = 0;
  }
  // for debugging, publish pwm
  pwm_pub.data = right_PWM;
  pwm_status.publish(&pwm_pub);
}

void left_motor(int pwm) {
  if (pwm > 0) {
    digitalWrite(INA_1,LOW);
    digitalWrite(INB_1,HIGH);
    analogWrite(PWM_1, pwm);
  } else {
    digitalWrite(INA_1,HIGH);
    digitalWrite(INB_1,LOW);
    analogWrite(PWM_1, pwm);
  }
}

void right_motor(int pwm) {
  if (pwm > 0) {
    digitalWrite(INA_2,HIGH);
    digitalWrite(INB_2,LOW);
    analogWrite(PWM_2, pwm);
  } else {
    digitalWrite(INA_2,LOW);
    digitalWrite(INB_2,HIGH);
    analogWrite(PWM_2, pwm);
  }
}

void stop_moving() {
  //Left Motor
  digitalWrite(INA_1,HIGH);
  digitalWrite(INB_1,HIGH);
  analogWrite(PWM_1,0);
  //Right Motor
  digitalWrite(INA_2,HIGH);
  digitalWrite(INB_2,HIGH);
  analogWrite(PWM_2,0);
}

void setup() {
  // put your setup code here, to run once:
  //Setting Left Motor pin as OUTPUT
  pinMode(INA_1,OUTPUT);
  pinMode(INB_1,OUTPUT);
  pinMode(PWM_1,OUTPUT);
  //Setting Right Motor pin as OUTPUT
  pinMode(INA_2,OUTPUT);
  pinMode(INB_2,OUTPUT);
  pinMode(PWM_2,OUTPUT);
  nh.initNode();
  nh.subscribe(cmd_sub);
  nh.advertise(pwm_status);
}

void loop() {
  // put your main code here, to run repeatedly: 
  // move the motors
  left_motor(left_PWM);
  right_motor(right_PWM);
  delay(100);
  nh.spinOnce();
  
}
