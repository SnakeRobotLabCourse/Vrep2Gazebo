# How to use Matlab to control the RoboSnake through ROS

## Setup Matlab

You will need the robotics system toolbox for this to work.

## Fix Matlab

(Tested with Matlab R2016a, Gazebo 7, ROS kinetic)

Trying to get Matlab to work with Gazebo 7 according to [this Mathworks guide](https://de.mathworks.com/help/robotics/examples/read-model-and-simulation-properties-from-gazebo.html) will probably result in
```
[ERROR] ServiceClientHandshakeHandler - Service client handshake failed: client wants service /gazebo/get_model_state to have md5sum af0f702011820738976b120226dc9d96, but it has 4c515e936d3319c9610c559c60bfc3d4. Dropping connection.
```

The reason for this is that a ROS message definition for gazebo was [slightly changed  for ROS jade](https://github.com/ros-simulation/gazebo_ros_pkgs/commit/30514e147b867ff46d7df4b0c230151fd3d0ee5f#diff-c00a545a26005b802d1b18184e244fdd) but the matlab library wasn't updated yet.

To solve this problem you have to replace Matlabs own message definition jar in *_<matlabdir>_/java/jarext/rosjava/rosjava_messages.jar* with the one provided in [bugfix/](bugfix/)

See [this thread with our answer](https://de.mathworks.com/matlabcentral/answers/278917-rossvcclient-handshake-fail-in-matlab2015b-ros-jade-gazebo-7) for more details on how to do this manually

## Adapt init.m to your system
Make sure that `ipaddress='192.168.42.42';` contains the IP address that the ROS server is running on.

## Start the gait
First run init.m to setup the connection to ROS / Gazebo / The RobotSnake

Then start eg. perform_gait_simple.m to run a gait and test the connection.

Use `rosshutdown` or cleanup.m to stop everything and re-run init.m in case of problems.



# Documentation 

## setJointTargetPosition.m
The defined function takes a joint (int) and a position (float, radians) and sets the given joints target angle to the value.
`robosnake_control` must be running for this to work!

The rospublishers will be created if not existing and will be cached to avoid reconnections.
Without caching the publishing might not work.
For the same reason, the first message sent might not have an effect.

## gait_rolling.m
This function should be called with a timer to perform the rolling gait.
It uses global values defined in init.m, so this needs to be run first.
By subscribing to the rostopic `/clock`, the gait is synchronized to the simulation time of gazebo.