ipaddress='192.168.184.130';
rosinit(ipaddress);
gazebo = ExampleHelperGazeboCommunicator();
snake=ExampleHelperGazeboSpawnedModel('11segment', gazebo);
[snakeLinks, snakeJoints] = getComponents(snake);