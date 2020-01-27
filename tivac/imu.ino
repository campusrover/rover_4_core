#include "Wire.h"

#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accelgyro;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Setup_MPU6050();
}

void loop() {
  // put your main code here, to run repeatedly: 
  Update_MPU6050();
}

void Setup_MPU6050()
{
  Wire.begin(3);
  Wire.setModule(3);
  Wire.begin();
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  //Serial.println(accelgyro.getDeviceID());
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connectionsuccessful" : "MPU6050 connection failed");
}

void Update_MPU6050()
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // display tab-separated accel/gyro x/y/z values
  Serial.print("i");Serial.print("\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.print(gz); Serial.print("\t");
  Serial.print("\n");
}
