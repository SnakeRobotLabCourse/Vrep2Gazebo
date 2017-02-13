#include <math.h>
#include "sidewinding.hpp"
#include <boost/bind.hpp>


using namespace gazebo;

SidewindingGait::SidewindingGait()
{
	C = {0.0, 0.0};
	A = {toRad(40), toRad(40)};
	bigOmega = {M_PI * 2.0 / 3.0, M_PI * 2.0 / 3.0};
	smallOmega = {4.0, 4.0};
	rho = M_PI/4.0;
}

std::vector<ros::SubscribeOptions> SidewindingGait::getSubscribers(ros::CallbackQueue *queue, std::string modelName) 
{
	ros::SubscribeOptions so1 = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
	      "/" + modelName + "/sidewinding/set_C",
	      1,
	      boost::bind(&SidewindingGait::OnRosMsgC, this, _1),
	      ros::VoidPtr(), queue);
	ros::SubscribeOptions so2 = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
	      "/" + modelName + "/sidewinding/set_A",
	      1,
	      boost::bind(&SidewindingGait::OnRosMsgA, this, _1),
	      ros::VoidPtr(), queue);
	ros::SubscribeOptions so3 = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
	      "/" + modelName + "/sidewinding/set_BigOmega",
	      1,
	      boost::bind(&SidewindingGait::OnRosMsgBigOmega, this, _1),
	      ros::VoidPtr(), queue);
	ros::SubscribeOptions so4 = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
	      "/" + modelName + "/sidewinding/set_SmallOmega",
	      1,
	      boost::bind(&SidewindingGait::OnRosMsgSmallOmega, this, _1),
	      ros::VoidPtr(), queue);
	ros::SubscribeOptions so5 = ros::SubscribeOptions::create<std_msgs::Float32>(
	      "/" + modelName + "/sidewinding/set_rho",
	      1,
	      boost::bind(&SidewindingGait::OnRosMsgRho, this, _1),
	      ros::VoidPtr(), queue);

	std::vector<ros::SubscribeOptions> options;
	options.push_back(so1);
	options.push_back(so2);
	options.push_back(so3);
	options.push_back(so4);
	options.push_back(so5);
	
	return options;
}

void SidewindingGait::OnRosMsgC(const std_msgs::Float32MultiArrayConstPtr &_msg)
{
	if (_msg->data.size() != 2) {
		std::cerr << "Received C message with wrong size: " << _msg->data.size() << "\n";
		return;
	}
	C = {toRad(_msg->data[0]), toRad(_msg->data[1])};
}

void SidewindingGait::OnRosMsgA(const std_msgs::Float32MultiArrayConstPtr &_msg)
{
	if (_msg->data.size() != 2) {
		std::cerr << "Received A message with wrong size: " << _msg->data.size() << "\n";
		return;
	}
	A = {toRad(_msg->data[0]), toRad(_msg->data[1])};
}

void SidewindingGait::OnRosMsgBigOmega(const std_msgs::Float32MultiArrayConstPtr &_msg)
{
	if (_msg->data.size() != 2) {
		std::cerr << "Received bigOmega message with wrong size: " << _msg->data.size() << "\n";
		return;
	}
	bigOmega = {_msg->data[0], _msg->data[1]};
}

void SidewindingGait::OnRosMsgSmallOmega(const std_msgs::Float32MultiArrayConstPtr &_msg)
{
	if (_msg->data.size() != 2) {
		std::cerr << "Received smallOmega message with wrong size: " << _msg->data.size() << "\n";
		return;
	}
	smallOmega = {_msg->data[0], _msg->data[1]};
}

void SidewindingGait::OnRosMsgRho(const std_msgs::Float32ConstPtr &_msg)
{
	rho = _msg->data;
}

std::vector<double> SidewindingGait::getAngle(double dt, int numberJoints)
{
	t += dt;
	std::vector<double> result;
	for(unsigned i = 0; i < numberJoints; i++)
	{
		if (i % 2 == 0)
		{
			result.push_back(C[1] + A[1] * (((1.0 + i)/numberJoints) * 0.9 + 0.1) * sin(smallOmega[1] * t + i * bigOmega[1]));
		}
		else
		{
			result.push_back(C[0] + A[0] * (((1.0 + i)/numberJoints) * 0.9 + 0.1) * cos(smallOmega[0] * t + i * bigOmega[0] + rho));
		}
	}
	return result;
}
