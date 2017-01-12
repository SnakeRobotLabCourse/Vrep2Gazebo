ipaddress='192.168.184.4';
rosinit(ipaddress);
gazebo = ExampleHelperGazeboCommunicator();
snake=ExampleHelperGazeboSpawnedModel('robosnake', gazebo);
[snakeLinks, snakeJoints] = getComponents(snake);