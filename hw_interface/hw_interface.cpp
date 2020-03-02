#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <math.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot() 
 { 
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_a("left_wheel", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_a);

   hardware_interface::JointStateHandle state_handle_b("right_wheel", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_b);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("left_wheel"), &cmd[0]);
   jnt_vel_interface.registerHandle(pos_handle_a);

   hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("right_wheel"), &cmd[1]);
   jnt_vel_interface.registerHandle(pos_handle_b);

   registerInterface(&jnt_vel_interface);

   // setup ros node???
   left_wheel_vel_pub = nh.advertise<std_msgs::Float32>("left_wheel_vel", 1);
   right_wheel_vel_pub = nh.advertise<std_msgs::Float32>("right_wheel_vel", 1);

   left_encoder_sub = nh.subscribe("encoder_left", 1, &MyRobot::left_enc_cb, this);
   right_encoder_sub = nh.subscribe("encoder_right", 1, &MyRobot::right_enc_cb, this);
  }

  void read(const ros::Duration &period)
  {
    /*
    * This function needs to recieve encoder data from the embedded robot hardware, update pos and vel
    */
    double ang_distance_left = wheel_angles[0];
    double ang_distance_right = wheel_angles[1];
    pos[0] += ang_distance_left;
    vel[0] += ang_distance_left / period.toSec();
    pos[1] += ang_distance_right;
    vel[1] += ang_distance_right / period.toSec();
  }

  void write()
  {
    /*
    * This function should read cmd and publish speed commands to the embedded robot hardware
    *
    */
    double diff_ang_speed_left = cmd[0];
    double diff_ang_speed_right = cmd[1];
    // convert radians/ sec to meters/ sec
    diff_ang_speed_left *= wheel_radius * M_PI;
    diff_ang_speed_right *= wheel_radius * M_PI;
    // publish commands
    std_msgs::Float32 left_wheel_vel_msg;
    std_msgs::Float32 right_wheel_vel_msg;
    left_wheel_vel_msg.data = diff_ang_speed_left;
    right_wheel_vel_msg.data = diff_ang_speed_right;
    left_wheel_vel_pub.publish(left_wheel_vel_msg);
    right_wheel_vel_pub.publish(right_wheel_vel_msg);

  }

  void left_enc_cb(const std_msgs::Int64& msg)
  {
    /*
    * Encoder callback receives raw encoder value from embedded robot board. 
    * should convert value from ticks to a change in wheel angle in radians
    */
    int delta_ticks = encoder_difference(msg.data, ticks_left);
    wheel_angles[0] = (delta_ticks / 800) * 2; // ticks / 800 is the fraction of a circle, so twice that is the number of radians in that protion of the circle
    ticks_left = msg.data;

  }

  void right_enc_cb(const std_msgs::Int64& msg)
  {
    /*
    * same thing here as left but other side
    */ 
    int delta_ticks = encoder_difference(msg.data, ticks_left);
    wheel_angles[1] = (delta_ticks / 800) * 2;
    ticks_right = msg.data;
  }

  int encoder_difference(const int ticks, const int prev_ticks)
  {
    /*
    * See src/odom_tf.py for the function of the same name for explanation for how this works. 
    */
    int max_ticks = 4294967296;
    int delta_ticks = ticks - prev_ticks;
    int extended_ticks;
    if (abs(delta_ticks) > max_ticks)
    {
        if (ticks > prev_ticks)
        {
            extended_ticks = -max_ticks - (max_ticks - ticks);
        }
        else
        {
            extended_ticks = max_ticks + (ticks + max_ticks);
        }
        delta_ticks = extended_ticks - prev_ticks;
    }
    return delta_ticks;
  }

  ros::Time get_time() {
    prev_update_time = curr_update_time;
    curr_update_time = ros::Time::now();
    return curr_update_time;
  }

  ros::Duration get_period() {
    return curr_update_time - prev_update_time;
  }


  ros::NodeHandle nh;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
  double wheel_angles[2];
  ros::Time curr_update_time, prev_update_time;
  float wheel_radius = 0.045; // meters
  int ticks_left, ticks_right;

  // TODO: publishers for wheel velicities to robot
  ros::Publisher left_wheel_vel_pub;
  ros::Publisher right_wheel_vel_pub;

  // TODO: subscribers for encoders from robot
  ros::Subscriber left_encoder_sub;
  ros::Subscriber right_encoder_sub;
};

int main(int argc, char **argv)
{
    /*
    * Main loop of the hardware interface.
    */
    ros::init(argc, argv, "hw_interface");
    MyRobot robot;
    controller_manager::ControllerManager cm(&robot);
    ros::Time time_now;
    ros::Duration period_now;
    ros::Rate sleep_rate(10);
    

    while (true)
    {
        time_now = robot.get_time();
        period_now = robot.get_period(); 
        robot.read(period_now);
        cm.update(time_now, period_now);
        robot.write();
        sleep_rate.sleep();
    }
}