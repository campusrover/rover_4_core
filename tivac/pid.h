 
#include <ros.h>
#include <std_msgs/Float32.h>
#include "motor.h"
//#include "tivac.h"


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
double PROPORTIONAL_GAIN = 5;
double INTEGRAL_GAIN = 3;
double DERIVITIVE_GAIN = 3;



