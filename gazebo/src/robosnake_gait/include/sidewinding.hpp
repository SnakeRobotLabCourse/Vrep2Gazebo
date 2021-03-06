#ifndef SIDEWINDINGGAIT_HPP
#define SIDEWINDINGGAIT_HPP
#include <vector>
#include <string> 
#include "gait.hpp"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

namespace gazebo 
	{
	class SidewindingGait : public Gait {
		public: 
			SidewindingGait();
			
			std::vector<ros::SubscribeOptions> getSubscribers(ros::CallbackQueue *queue, std::string modelName);
			std::vector<double> getAngle(double dt, int numberJoints);

			void OnRosMsgC(const std_msgs::Float32MultiArrayConstPtr &_msg);
			void OnRosMsgA(const std_msgs::Float32MultiArrayConstPtr &_msg);
			void OnRosMsgBigOmega(const std_msgs::Float32MultiArrayConstPtr &_msg);
			void OnRosMsgSmallOmega(const std_msgs::Float32MultiArrayConstPtr &_msg);
			void OnRosMsgRho(const std_msgs::Float32ConstPtr &_msg);

		protected:
			std::vector<double> C;
			std::vector<double> A;
			std::vector<double> bigOmega;
			std::vector<double> smallOmega;
			double rho;
	};
}

#endif
