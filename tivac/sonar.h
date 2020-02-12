#include <ros.h>
#include <math.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Header.h>

// Sonar defines
#define Trig 10
#define Echo 9

// Sonar 
long duration, cm;

sensor_msgs::Range sonar;
ros::Publisher sonar_pub("sonar_front", &sonar);
std_msgs::Header sonar_head;

void Update_Ultra_Sonic();
void updateSonar();