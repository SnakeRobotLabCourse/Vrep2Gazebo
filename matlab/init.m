
% Initialize global Variables & IP-Address

ipaddress='192.168.184.4';
rosinit(ipaddress);

global gazebo;
global snake;
global snakeLinks;
global snakeJoints;

gazebo = ExampleHelperGazeboCommunicator();
snake = ExampleHelperGazeboSpawnedModel('robosnake', gazebo);
[snakeLinks, snakeJoints] = getComponents(snake);