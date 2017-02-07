#ifndef GAIT_HPP
#define GAIT_HPP
#include <string> 
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

namespace gazebo
{
	class Gait{
		protected: 
			std::vector<double> speed;
			std::vector<double> a_h;
			std::vector<double> a_v;
			std::vector<double> p_h;
			std::vector<double> p_v;
			double s;
			double t;

			double toRad(double deg);
	
		public: 
			virtual double getAngle(double dt, int joint) = 0;
			virtual std::vector<ros::SubscribeOptions> getSubscribers(ros::CallbackQueue *queue, std::string modelName) = 0;
	};
}

#endif
