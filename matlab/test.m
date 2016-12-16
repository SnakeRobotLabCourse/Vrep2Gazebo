j0 = rospublisher('/11segment/joint_0', 'std_msgs/Float32');
msg = rosmessage('std_msgs/Float32');
msg.Data = 0.7;

send(j0, msg);