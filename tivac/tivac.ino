
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
  float left_vel = 1.0 * linear_vel + angular_vel * width / 2.0;
  float right_vel = 1.0 * linear_vel - angular_vel * width / 2.0;
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
    left_PWM = -(left_vel / MIN_LINEAR_VEL) * 255;
  } else {
    left_PWM = 0;
  }
  // convert to PWM (right)
  if (right_vel > 0) {
    right_PWM = (right_vel / MAX_LINEAR_VEL) * 255;
  } else if (right_vel < 0) {
    right_PWM = -(right_vel / MIN_LINEAR_VEL) * 255;
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
    analogWrite(PWM_1, -pwm); // a negative pwm loops back around (?) so passing -2 will send a pwm of 253. --2 is 2 though (obviously)
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
    analogWrite(PWM_2, -pwm);
  }
}


void do_Left_Encoder()
{
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);
  // read the input pin
  Left_Encoder_Ticks -= LeftEncoderBSet ? +1 : -1;
  if (Left_Encoder_Ticks > ENCODER_MAX) {
    Left_Encoder_Ticks = ENCODER_MIN + (Left_Encoder_Ticks - ENCODER_MAX);
  } else if (Left_Encoder_Ticks < ENCODER_MIN) {
    Left_Encoder_Ticks = ENCODER_MAX - (ENCODER_MIN - Left_Encoder_Ticks);
  }
}

void do_Right_Encoder()
{
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);
  // read the input pin
  Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;
  if (Right_Encoder_Ticks > ENCODER_MAX) {
    Right_Encoder_Ticks = ENCODER_MIN + (Right_Encoder_Ticks - ENCODER_MAX);
  } else if (Right_Encoder_Ticks < ENCODER_MIN) {
    Right_Encoder_Ticks = ENCODER_MAX - (ENCODER_MIN - Right_Encoder_Ticks);
  }
}


void Update_Ultra_Sonic()
{
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  duration = pulseIn(Echo, HIGH);
  // convert the time into a distance
  cm = duration / 14.5;
}

void setup() {
  // put your setup code here, to run once:
  // Motors
  //Setting Left Motor pin as OUTPUT
  pinMode(INA_1,OUTPUT);
  pinMode(INB_1,OUTPUT);
  pinMode(PWM_1,OUTPUT);
  //Setting Right Motor pin as OUTPUT
  pinMode(INA_2,OUTPUT);
  pinMode(INB_2,OUTPUT);
  pinMode(PWM_2,OUTPUT);
  // Quadrature encoders
  // Left encoder
  pinMode(Left_Encoder_PinA, INPUT_PULLUP); // sets pin A as input
  pinMode(Left_Encoder_PinB, INPUT_PULLUP); // sets pin B as input
  attachInterrupt(Left_Encoder_PinA, do_Left_Encoder, RISING);
  // Right encoder
  pinMode(Right_Encoder_PinA, INPUT_PULLUP); // sets pin A as input
  pinMode(Right_Encoder_PinB, INPUT_PULLUP); // sets pin B as input
  attachInterrupt(Right_Encoder_PinA, do_Right_Encoder, RISING);
  // Sonar 
  pinMode(Trig, OUTPUT); // trigger pin
  pinMode(Echo, INPUT); // echo pin
  sonar.radiation_type = sonar.ULTRASOUND;
  sonar.field_of_view = 0.261799; // radians, adafruit's website cites a 15 degree FOV
  sonar.min_range = 0.02;
  sonar.max_range = 4; // some data sheets say 3m is max range, most others say 4, but also that measurements are most accurate < 250cm
  sonar_head.frame_id = "sonar"; // this is an assumption, can be changed later
  sonar_head.seq = 0;
  sonar.header = sonar_head;
  // ROS
  nh.initNode();
  nh.subscribe(cmd_sub);
  nh.advertise(pwm_status);
  nh.advertise(left_enc_pub);
  nh.advertise(right_enc_pub);
  nh.advertise(sonar_pub);
}

void loop() {
  // put your main code here, to run repeatedly: 
  // move the motors
  left_motor(left_PWM);
  right_motor(right_PWM);
  // update encoders and publish 
  enc_l.data = Left_Encoder_Ticks;
  enc_r.data = Right_Encoder_Ticks;
  left_enc_pub.publish(&enc_r);  // objects are swapped because the wheels on the robot are reversed?
  right_enc_pub.publish(&enc_l);
  // Update sonar, then publish
  Update_Ultra_Sonic();
  sonar.range = cm / 100.0;
  sonar_pub.publish(&sonar);
  sonar_head.seq++;
  delay(100);
  nh.spinOnce();
  
}
