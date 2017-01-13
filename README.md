# Vrep2Gazebo

## Dependencies

```
sudo apt install 
ros-kinetic-gazebo-ros-control
ros-kinetic-effort-controllers
ros-kinetic-joint-state-controller
```

## Build
Go to `gazebo`-folder and build using
```
catkin_make
```
(You might need to delete `build/` and `devel/` before)

## Run
Make sure you have
```
source devel/setup.bash
```

Start Gazebo with the model
```
roslaunch robosnake_gazebo robosnake.launch
```

Start the controller
```
roslaunch robosnake_control robosnake_control.launch
```
