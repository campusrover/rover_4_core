# Campus Rover 4 Prototype + final core

"low level" code for the campus rover 4 platform

**This is the most up-to-date branch of rover 4 core, with the best odometry/ move base/ PID control** please read README.md on branch master for more info about this package

## Camera

Robot uses Orbbec astra pro camera
git source [here](https://github.com/orbbec/ros_astra_camera)
install instructions [here](http://wiki.ros.org/astra_camera)

when invoking catkin_make on this package, use the `j1` argument. Also turn DFILTER OFF

## Reading

These are some sites that helped to get the diff drive controller working

* [ros answers thread on getting diff drive controller to work](https://answers.ros.org/question/345887/diff_drive_controller-not-subscribing-to-cmd_vel/)
* [ros answers thread on linking cpp files in cmake](https://answers.ros.org/question/345645/using-diff_drive-problem-with-our-hw_interface/)
* [ros wiki hardware interface tutorial, simple and clean](http://wiki.ros.org/ros_control/Tutorials/Create%20your%20own%20hardware%20interface)
* [main ros control wiki page](http://wiki.ros.org/ros_control)

other sources that were good to reference (google yourself)

* the turtlebot3 core source code
* ros control boilerplate
* anything on urdf creation
* the entire turtlebot3 package is a great resource, read through their launch files and you'll learn a lot
