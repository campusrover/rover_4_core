#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#include <ros.h>
#include <math.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

// IMU
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

sensor_msgs::Imu imu;
ros::Publisher imu_pub("imu", &imu);

void updateIMU();
