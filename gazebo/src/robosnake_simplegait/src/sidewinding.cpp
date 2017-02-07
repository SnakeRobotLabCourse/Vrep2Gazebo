#include <math.h>
#include "sidewinding.hpp"
#include <boost/bind.hpp>


using namespace gazebo;

SidewindingGait::SidewindingGait()
{
	C = {0, 0};
	A = {toRad(40), toRad(40)};
	bigOmega = {2.0 / 3.0 * M_PI, 2.0 / 3.0 * M_PI};
	smallOmega = {4.0, 4.0};
	rho = M_PI/4.0;
}

std::vector<ros::SubscribeOptions> SidewindingGait::getSubscribers(ros::CallbackQueue *queue, std::string modelName) 
{
	/**ros::SubscribeOptions so1 = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
	      "/" + modelName + "/simplegait/set_amplitude",
	      1,
	      boost::bind(&SlitheringGait::OnRosMsgAmplitude, this, _1),
	      ros::VoidPtr(), queue);
	ros::SubscribeOptions so2 = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
	      "/" + modelName + "/simplegait/set_phase",
	      1,
	      boost::bind(&SlitheringGait::OnRosMsgPhase, this, _1),
	      ros::VoidPtr(), queue);**/
	std::vector<ros::SubscribeOptions> options;
	/**options.push_back(so1);
	options.push_back(so2);
	**/
	return options;
}

/**void SidewindingGait::OnRosMsgAmplitude(const std_msgs::Float32MultiArrayConstPtr &_msg)
{
	if (_msg->data.size() != 4) {
		std::cerr << "Received amplitude message with wrong size: " << _msg->data.size() << "\n";
		return;
	}
	a_v = {toRad(_msg->data[0]), toRad(_msg->data[1])};
	a_h = {toRad(_msg->data[2]), toRad(_msg->data[3])};
}

void SidewindingGait::OnRosMsgPhase(const std_msgs::Float32MultiArrayConstPtr &_msg)
{
	if (_msg->data.size() != 4) {
		std::cerr << "Received phase message with wrong size: " << _msg->data.size() << "\n";
		return;
	}
	p_v = {toRad(_msg->data[0]), toRad(_msg->data[1])};
	p_h = {toRad(_msg->data[2]), toRad(_msg->data[3])};
}**/

double SidewindingGait::getAngle(double dt, int joint)
{
	if (joint % 2)
	{
		return C[0] + A[0] * (((1.0 + joint)/11.0) * 0.9 + 0.1) * cos(smallOmega[0] * t * (joint) * bigOmega[0] + rho);
	}
	else
	{
		return C[1] + A[1] * (((1.0 + joint)/11.0) * 0.9 + 0.1) * sin(smallOmega[1] * t * (joint) * bigOmega[1]);
	}	
}