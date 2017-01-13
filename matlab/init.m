ipaddress='192.168.30.130';
rosinit(ipaddress);

global gazebo;
global snake;
global snakeLinks;
global snakeJoints;

gazebo = ExampleHelperGazeboCommunicator();
snake=ExampleHelperGazeboSpawnedModel('robosnake', gazebo);
[snakeLinks, snakeJoints] = getComponents(snake);