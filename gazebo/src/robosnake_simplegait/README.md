This is a gazebo plugin that controls the gait directly.

To use it, declare ``<plugin>...</plugin>`` in your model sdf.

Commands are:
rostopic pub /robosnake/gait_amplitude std_msgs/Float32MultiArray "layout:
   dim: []
   data_offset: 0
data: [10, 10, 20, 20]"

rostopic pub /robosnake/gait_phase std_msgs/Float32MultiArray "layout:
   dim: []
   data_offset: 0
data: [10, 10, 20, 20]"
