#ifndef GAIT_HPP
#define GAIT_HPP
#include <string> 
#include "ros/callback_queue.h"
#include <gazebo/physics/physics.hh>
#include "ros/subscribe_options.h"

namespace gazebo
{
	class Gait{
		protected:
			double t;

			double toRad(double deg);
	
		public: 
			virtual std::vector<double> getAngle(double dt, int numberJoints) = 0;
			virtual std::vector<ros::SubscribeOptions> getSubscribers(ros::CallbackQueue *queue, std::string modelName) = 0;
	};
}

#endif
