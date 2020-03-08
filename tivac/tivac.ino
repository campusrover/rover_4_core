#include "tivac.h"
#include "battery.h"
#include "imu.h"
#include "motor.h"
#include "sonar.h"
#include "pid.h"


void cmd_cb(const geometry_msgs::Twist& msg) {
  // twist to velocity based on https://github.com/PacktPublishing/Learning-Robotics-using-Python-Second-Edition/blob/master/chapter_8_code/chefbot_bringup/scripts/twist_to_motors.py
  // width of robot
  double width = 0.26; //meters
  // linear component
  float linear_vel = msg.linear.x;
  // angular component
  float angular_vel = msg.angular.z;
  // combine values to get individual wheel velocities
  left_vel = 1.0 * linear_vel - angular_vel * width / 2.0;
  right_vel = 1.0 * linear_vel + angular_vel * width / 2.0;
  
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
  left_PWM = (left_vel / MAX_LINEAR_VEL) * 255;
  // convert to PWM (right)
  right_PWM = (right_vel / MAX_LINEAR_VEL) * 255;
}

void left_motor(int pwm) {
  if (pwm > 0) {
    digitalWrite(INA_1,HIGH);
    digitalWrite(INB_1,LOW);
    analogWrite(PWM_1, pwm);
  } else {
    digitalWrite(INA_1,LOW);
    digitalWrite(INB_1,HIGH);
    analogWrite(PWM_1, -pwm); // a negative pwm loops back around (?) so passing -2 will send a pwm of 253. --2 is 2 though (obviously)
  }
}

void right_motor(int pwm) {
  if (pwm > 0) {
    digitalWrite(INA_2,LOW);
    digitalWrite(INB_2,HIGH);
    analogWrite(PWM_2, pwm);
  } else {
    digitalWrite(INA_2,HIGH);
    digitalWrite(INB_2,LOW);
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

int encoder_difference(const int ticks, const int prev_ticks)
  {
    /*
    * See src/odom_tf.py for the function of the same name for explanation for how this works. 
    */
    int delta_ticks = ticks - prev_ticks;
    int extended_ticks;
    if (abs(delta_ticks) > ENCODER_MAX)
    {
        extended_ticks = (ticks > prev_ticks) ? ENCODER_MIN - (ENCODER_MAX - ticks):ENCODER_MAX + (ticks + ENCODER_MAX);
        delta_ticks = extended_ticks - prev_ticks;
        
    }
    return delta_ticks;
  }


void Update_Ultra_Sonic()
{
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  duration = pulseIn(Echo, HIGH, 10000);
  // convert the time into a distance
  cm = duration / 58; // / 29 / 2
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
  time_last = nh.now().toSec();
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
  sonar_head.frame_id = "sonar_link"; // this is an assumption, can be changed later
  sonar_head.seq = 0;
  sonar.header = sonar_head;
  // IMU
  imu.header.frame_id = "imu";
  imu.header.seq = 0;
  //This wire setup KILLS rosserial communication, from 8.6hz to 0.4hz
  Wire.begin(3);
  Wire.setModule(3);
  Wire.begin();
  
  accelgyro.initialize();
  accelgyro.setXGyroOffset(-15);
  accelgyro.setYGyroOffset(-115);
  accelgyro.setZGyroOffset(-35);
  accelgyro.setXAccelOffset(-1035);
  accelgyro.setYAccelOffset(-1690);
  accelgyro.setZAccelOffset(-1145);

  // ROS
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(cmd_sub);
  
  nh.subscribe(p_gain_sub);
  nh.subscribe(i_gain_sub);
  nh.subscribe(d_gain_sub);
  
  nh.advertise(left_enc_pub);
  nh.advertise(right_enc_pub);
  nh.advertise(sonar_pub);
  nh.advertise(imu_pub);
  nh.advertise(vel_left);
  nh.advertise(vel_right);
}

void loop() {
  
  updateMotors();
  
  updateSonar();
 
  updateIMU();
  
  delay(100);
  nh.spinOnce();
}


void updateMotors() {
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

  /* working snippet - do not remove until full PID is working
  // calculate pwm to send to motors. if target vel = 0, send 0 to eliminate motor whine. otherwise, scale according to error. 
  left_PWM_out = (left_vel == 0) ? 0:prev_left_PWM + (left_error > 0 ? ceil(left_error):floor(left_error));  // equivalent to P controller w/ gain of 1
  right_PWM_out = (right_vel == 0) ? 0:prev_right_PWM + (right_error > 0 ? ceil(right_error):floor(right_error));
  */

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

  left_PWM_out = prev_left_PWM + (left_PID_sum > 0 ? ceil(left_PID_sum):floor(left_PID_sum)); 
  right_PWM_out = prev_right_PWM + (right_PID_sum > 0 ? ceil(right_PID_sum):floor(right_PID_sum)); 

  // eliminate motor whine 
  left_PWM_out = (left_vel == 0 && abs(left_PWM_out < 10)) ? 0:left_PWM_out;
  right_PWM_out = (right_vel == 0 && abs(right_PWM_out < 10)) ? 0:right_PWM_out;

  prev_left_PWM = left_PWM_out;
  prev_right_PWM = right_PWM_out;
  prev_left_encoder_ticks = Left_Encoder_Ticks;
  prev_right_encoder_ticks = Right_Encoder_Ticks;
  // move the motors
  left_motor(left_PWM_out);
  right_motor(right_PWM_out);
  // update encoders and publish 
  enc_l.data = Left_Encoder_Ticks;
  enc_r.data = Right_Encoder_Ticks;
  left_enc_pub.publish(&enc_l);  
  right_enc_pub.publish(&enc_r);
  RVEL.data = right_vel_actual;
  LVEL.data = left_vel_actual;
  vel_left.publish(&LVEL);
  vel_right.publish(&RVEL);
  // push time back for next loop
  time_last = time_now;
}

void updateSonar() {
  // Update sonar, then publish
  Update_Ultra_Sonic();
  sonar.range = cm / 100.0;
  sonar_pub.publish(&sonar);
  sonar_head.seq++;
}

void updateIMU() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //angular velocity conversion
  auto f = [](int16_t av) {return av * (4000.0/65536.0) * (M_PI/180.0) / 25.0;};
  //linear acceleration conversation
  auto g = [](int16_t la) {return la * (8.0 / 65536.0);};
  
  imu.angular_velocity.x = f(gx);
  imu.angular_velocity.y = f(gy);
  imu.angular_velocity.z = f(gz);
  
  imu.linear_acceleration.x = g(ax);
  imu.linear_acceleration.y = g(ay);
  imu.linear_acceleration.z = g(az);
  
//@debug
//  Serial.print(imu.angular_velocity.x); Serial.print("\t");
//  Serial.print(imu.angular_velocity.y); Serial.print("\t");
//  Serial.print(imu.angular_velocity.z); Serial.print("\t");
//  Serial.print(imu.linear_acceleration.x); Serial.print("\t");
//  Serial.print(imu.linear_acceleration.y); Serial.print("\t");
//  Serial.println(imu.linear_acceleration.z);

  imu_pub.publish(&imu);
  imu.header.seq++;
}

void update_battery() {
  // may be useful in future, not necessary yet...
  // https://www.instructables.com/id/Arduino-Battery-Voltage-Indicator/ gives a brief example of how to measure battery voltage 
  battery_level = analogRead(PC_4) * (5.00 / 1023.00) * 2;
  Serial.println(battery_level);
}
