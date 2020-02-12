# Campus Rover 4 Prototype + final core

"low level" code for the campus rover 4 platform

### Camera
Robot uses Orbbec astra pro camera
git source [here](https://github.com/orbbec/ros_astra_camera)
install instructions [here](http://wiki.ros.org/astra_camera)

when invoking catkin_make on this package, use the `j1` argument. Also turn DFILTER OFF

to run the energiaIDE, run using the alias
> $ energiaIDE

Connect to the robot using ssh by running
> $ ssh robot@robot.dyn.brandeis.edu

In the robot, run roscore
> $ roscore

To request connection via rosserial, run the following script in the robot shell
> $ tivac-serial
