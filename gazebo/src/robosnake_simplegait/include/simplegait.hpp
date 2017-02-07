#ifndef SIMPLEGAIT_HPP
#define SIMPLEGAIT_HPP
#include <vector>
#include <string> 
#include "gait.hpp"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32MultiArray.h"

namespace gazebo 
	{
	class SimpleGait : public Gait {
		public: 
			SimpleGait();
			
			std::vector<ros::SubscribeOptions> getSubscribers(ros::CallbackQueue *queue, std::string modelName);
			double getAngle(double dt, int joint);
			void OnRosMsgAmplitude(const std_msgs::Float32MultiArrayConstPtr &_msg);
			void OnRosMsgPhase(const std_msgs::Float32MultiArrayConstPtr &_msg);
	};
}

#endif
