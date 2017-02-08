# Using ROS & Gazebo


# In a virtual machine
As of 02/2017 we recommend using VMware over VirtualBox:
Gazebo needs hardware acceleration and will probably crash in Virtualbox

## Modifications made to the virtual machine
You might want to try these in case of errors and to improve things:
 * `echo "export SVGA_VGPU10=0" >> ~/.bashrc` (limit OPENGL-Version for hardware acceleration to work)
 * reduce swappiness to 10 (/etc/sysctl.conf) to speed up the VM
 * Install VMWare tools

# Installation & Dependencies
 * Install Gazebo: `sudo curl -ssL http://get.gazebosim.org | sh`
 * Install ROS Kinetic according to [http://wiki.ros.org/kinetic/Installation/Ubuntu]([http://wiki.ros.org/kinetic/Installation/Ubuntu])
 * Make sure `ros-kinetic-gazebo-ros-pkgs` is already installed
 * set ros envs (add to `~/.bashrc`):
   
		ROS_MASTER_URI=http://192.168.42.142:11311
		ROS_HOSTNAME=192.168.42.142
	
	(Include the local IP-Address of the VM, of course)
 * further dependencies:
 
		sudo apt install 
		ros-kinetic-gazebo-ros-control
		ros-kinetic-effort-controllers
		ros-kinetic-joint-state-controller


# Repository structure
This is a catkin-workspace containing multiple ROS-packages

 * `robosnake_description` Contains all Robosnake Models & Meshes
 * `robosnake_gazebo` Contains the world and the ROS launch file for the Gazebo-simulation
 * `robosnake_control` Contains a ROS-PID-Controller setup for the robosnake to controll the joints individually
 * `robosnake_simplegait` Contains a Gazebo-Plugin to control a simple gait via ROS ([more info](src/robosnake_simplegait))


# Building the repository

Inside the `gazebo`-folder, execute the following:
```
catkin_make
```
In case of problems:
 * Try deleting `build/` and `devel/` before
 * Try deleting `src/CMakeLists.txt` and executing `catkin_init_workspace` in `src/`


# Running the Code

Make sure you have
```
source devel/setup.bash
```

Start Gazebo with the model
```
roslaunch robosnake_gazebo robosnake.launch
```
(just restart if gazebo crashes)

If you want to control the joints individually via ROS (e.g. from Matlab), also start the controller
```
roslaunch robosnake_control robosnake_control.launch
```

