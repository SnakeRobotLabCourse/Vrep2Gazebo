This is a gazebo plugin that controls the gait directly.

To use it, declare ``<plugin>...</plugin>`` in your model sdf.

Commands are:
 - ``rostopic pub /robosnake/gait/start std_msgs/Empty``

 - ``rostopic pub /robosnake/simplegait/set_amplitude std_msgs/Float32MultiArray "layout:
   dim: []
   data_offset: 0
data: [10, 10, 20, 20]"``

 - ``rostopic pub /robosnake/simplegait/set_phase std_msgs/Float32MultiArray "layout:
   dim: []
   data_offset: 0
data: [10, 10, 20, 20]"``
