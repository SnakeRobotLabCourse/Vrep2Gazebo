#include <math.h>
#include "simplegait.hpp"
#include <boost/bind.hpp>


using namespace gazebo;

SimpleGait::SimpleGait()
{
	speed={-2.0, -2.0};
	a_h = {toRad(20.0), toRad(30.0)};
	a_v = {toRad(20.0), toRad(30.0)};
	p_v = {0.0, 0.0};
	p_h = {0.0, 0.0};
	t = 0.0;
	s = 0.0;
}

std::vector<ros::SubscribeOptions> SimpleGait::getSubscribers(ros::CallbackQueue *queue, std::string modelName) 
{
	ros::SubscribeOptions so1 = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
	      "/" + modelName + "/simplegait/set_amplitude",
	      1,
	      boost::bind(&SimpleGait::OnRosMsgAmplitude, this, _1),
	      ros::VoidPtr(), queue);
	ros::SubscribeOptions so2 = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
	      "/" + modelName + "/simplegait/set_phase",
	      1,
	      boost::bind(&SimpleGait::OnRosMsgPhase, this, _1),
	      ros::VoidPtr(), queue);
	std::vector<ros::SubscribeOptions> options;
	options.push_back(so1);
	options.push_back(so2);
	return options;
}

void SimpleGait::OnRosMsgAmplitude(const std_msgs::Float32MultiArrayConstPtr &_msg)
{
	if (_msg->data.size() != 4) {
		std::cerr << "Received amplitude message with wrong size: " << _msg->data.size() << "\n";
		return;
	}
	a_v = {toRad(_msg->data[0]), toRad(_msg->data[1])};
	a_h = {toRad(_msg->data[2]), toRad(_msg->data[3])};
}

void SimpleGait::OnRosMsgPhase(const std_msgs::Float32MultiArrayConstPtr &_msg)
{
	if (_msg->data.size() != 4) {
		std::cerr << "Received phase message with wrong size: " << _msg->data.size() << "\n";
		return;
	}
	p_v = {toRad(_msg->data[0]), toRad(_msg->data[1])};
	p_h = {toRad(_msg->data[2]), toRad(_msg->data[3])};
}

double SimpleGait::getAngle(double dt, int joint)
{
	if(joint == 0)
	{
		t += dt;
		if (t > 20.0)
		{
			s = t - 20.0;
			if (s > 1.0)
			{
				s = 1.0;
			}
		}
	}
	if (joint % 2 == 1)
	{
		return (a_v[0] * (1.0 - s) + a_v[1] * s) * sin(t * (speed[0] * (1.0 - s) + speed[1] * s) 
			+ (joint / 2 + 1)  * (p_v[0] * (1.0 - s) + p_v[1] * s));
	} else 
	{
		return (a_h[0] * (1.0 - s) + a_h[1] * s) * cos(t * (speed[0] * (1.0 - s) + speed[1] * s) 
			+ (joint / 2 + 1) * (p_h[0] * (1.0 - s) + p_h[1] * s));
	}	
}
