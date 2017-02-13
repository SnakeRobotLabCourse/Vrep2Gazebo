This is a gazebo plugin that controls the gait directly.

To use it, declare ``<plugin>...</plugin>`` in your model sdf.
(Already done here)

Commands are:
 - ``rostopic pub /robosnake/gait/start std_msgs/Empty``
 - ``rostopic pub /robosnake/gait/stop  std_msgs/Empty``

 - ``rostopic pub /robosnake/simplegait/set_amplitude std_msgs/Float32MultiArray "layout:
   dim: []
   data_offset: 0
data: [10, 10, 20, 20]"`` (amplitude vertical, then horizontal in deg)

 - ``rostopic pub /robosnake/simplegait/set_phase std_msgs/Float32MultiArray "layout:
   dim: []
   data_offset: 0
data: [10, 10, 20, 20]"`` (phase vertical, then horizontal in deg)


 - ``rostopic pub /robosnake/sidewinding/set_A std_msgs/Float32MultiArray "layout:
   dim: []
   data_offset: 0
data: [40, 40]"`` (in deg)

 - ``rostopic pub /robosnake/sidewinding/set_C std_msgs/Float32MultiArray "layout:
   dim: []
   data_offset: 0
data: [0, 0]"`` (in deg)

 - ``rostopic pub /robosnake/sidewinding/set_BigOmega std_msgs/Float32MultiArray "layout:
   dim: []
   data_offset: 0
data: [40, 40]"`` (in absolute numbers)

 - ``rostopic pub /robosnake/sidewinding/set_SmallOmega std_msgs/Float32MultiArray "layout:
   dim: []
   data_offset: 0
data: [2.1, 2.1]"`` (in absolute numbers)

 - ``rostopic pub /robosnake/sidewinding/set_rho std_msgs/Float32 "0.9"``(in deg)

 - ``rostopic pub /robosnake/gait/select std_msgs/String "sidewinding"``
 - ``rostopic pub /robosnake/gait/select std_msgs/String "simplegait"``
