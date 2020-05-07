# Campus Rover 4 Prototype + final core

"low level" code for the campus rover 4 platform

for a status report of this project, [read this page](https://campus-rover.gitbook.io/lab-notebook/cr-package/campus-rover-4/progress-report). 

### Camera
Robot uses Orbbec astra pro camera
git source [here](https://github.com/orbbec/ros_astra_camera)
install instructions [here](http://wiki.ros.org/astra_camera)

when invoking catkin_make on this package, use the `j1` argument. Also turn DFILTER OFF

# General First Steps to Start Up

to run the energiaIDE, we suggest creating an alias that points to the location of your energia execuable
> $ energiaIDE

Connect to the robot using ssh by running
> $ ssh robot@robot.dyn.brandeis.edu

In the robot, run roscore
> $ roscore

To request connection via rosserial, run the following script in the robot shell (note - this exists on the existing cr4 hardware, is not included on new models)
> $ tivac-serial
