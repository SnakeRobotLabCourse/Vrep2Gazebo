#ifndef _GAIT_PLUGIN_HH_
#define _GAIT_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include <vector>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "gait.hpp"
#include "simplegait.hpp"

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
	class SimpleGaitPlugin : public ModelPlugin
	{
		/// \brief Constructor
		public: 
			SimpleGaitPlugin() {}

			/// \brief The load function is called by Gazebo when the plugin is
			/// inserted into simulation
			/// \param[in] _model A pointer to the model that this plugin is
			/// attached to.
			/// \param[in] _sdf A pointer to the plugin's SDF element.
			virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
			{
				isRunning = 0;
				model = _model;
				joints = _model->GetJoints();
				// Setup PID controllers
				for(unsigned i = 0; i < joints.size(); i++){
					model->GetJointController()->SetPositionPID(joints[i]->GetScopedName(), common::PID(33.0, 1.2, 0.0));
				}
				model->GetJointController()->Update();
			
				// Initialize ros, if it has not already bee initialized.
				if (!ros::isInitialized())
				{
				  std::cerr << "Ros was not initialized, initializing...";
				  int argc = 0;
				  char **argv = NULL;
				  ros::init(argc, argv, "gazebo_client",
				      ros::init_options::NoSigintHandler);
				}

				// Create our ROS node. This acts in a similar manner to
				// the Gazebo node
				rosNode.reset(new ros::NodeHandle("gazebo_client"));

				// Create a named topic for start, stop and gait select, and subscribe to it.
				ros::SubscribeOptions soStart = ros::SubscribeOptions::create<std_msgs::Empty>(
				      "/" + model->GetName() + "/gait/start",
				      1,
				      boost::bind(&SimpleGaitPlugin::OnRosMsgStart, this, _1),
				      ros::VoidPtr(), &rosQueue);
				ros::SubscribeOptions soStop = ros::SubscribeOptions::create<std_msgs::Empty>(
				      "/" + model->GetName() + "/gait/stop",
				      1,
				      boost::bind(&SimpleGaitPlugin::OnRosMsgStop, this, _1),
				      ros::VoidPtr(), &rosQueue);
				ros::SubscribeOptions soGaitSel = ros::SubscribeOptions::create<std_msgs::String>(
				      "/" + model->GetName() + "/gait/select_gait",
				      1,
				      boost::bind(&SimpleGaitPlugin::OnRosMsgGaitSel, this, _1),
				      ros::VoidPtr(), &rosQueue);
					
				
				rosSubStart = rosNode->subscribe(soStart);
				rosSubStop = rosNode->subscribe(soStop);
				rosSubGaitSel = rosNode->subscribe(soGaitSel);

				// Spin up the queue helper thread.
				rosQueueThread = std::thread(std::bind(&SimpleGaitPlugin::QueueThread, this));

				// Bind Gait Polymorphic to SimpleGait
				SimpleGait* simpleGait = new SimpleGait();
				initGait(simpleGait);

				// Listen to the update event. This event is broadcast every
	      			// simulation iteration.
	      			updateConnection = event::Events::ConnectWorldUpdateBegin(
		  			boost::bind(&SimpleGaitPlugin::OnUpdate, this, _1));
			
			}

			// Called by the world update start event
	    		void OnUpdate(const common::UpdateInfo & /*_info*/)
	    		{
				// Get the simulation time and period
				gazebo::common::Time gz_time_now = model->GetWorld()->GetSimTime();
				ros::Time sim_time_ros (gz_time_now.sec, gz_time_now.nsec);
				ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

				if (isRunning){
					// Update joint positions
					for(unsigned i = 0; i < joints.size(); i++){
						double pos = gait->getAngle(sim_period.toSec(), i);
						model->GetJointController()->SetPositionTarget(joints[i]->GetScopedName(), pos);
					}
				}

				last_update_sim_time_ros_ = sim_time_ros;
	    		}
			// Called on world reset
			void Reset()
			{
			  // Reset timing variables to not pass negative update periods to controllers on world reset
			  last_update_sim_time_ros_ = ros::Time();
			}

			/// \brief Handle incoming messages from ROS
			void OnRosMsgStart(const std_msgs::EmptyConstPtr &_msg){
				isRunning = 1;
			}
			void OnRosMsgStop(const std_msgs::EmptyConstPtr &_msg){
				isRunning = 0;
			}
			void OnRosMsgGaitSel(const std_msgs::StringConstPtr &_msg){
				if (_msg->data.compare("simplegait"))
				{
					delete(gait);
					SimpleGait* simpleGait = new SimpleGait();
					initGait(simpleGait);
				}
			}

		
		private: 
			/// \brief ROS helper function that processes messages
			void QueueThread()
			{
				static const double timeout = 0.01;
				while (rosNode->ok())
				{	
					rosQueue.callAvailable(ros::WallDuration(timeout));
				}
			}
			
			/// \brief Sets up the new gait
			void initGait(Gait* gait){
				this->gait = gait;
				
				for(std::vector<ros::Subscriber>::iterator it = gaitSubscribers.begin(); it != gaitSubscribers.end(); ++it) {
					it->shutdown();				
				}
				gaitSubscribers.clear();

				std::vector<ros::SubscribeOptions> gaitSos = gait->getSubscribers(&rosQueue, model->GetName());
				for(std::vector<ros::SubscribeOptions>::iterator it = gaitSos.begin(); it != gaitSos.end(); ++it) {
 					gaitSubscribers.push_back(rosNode->subscribe(*it));
				}
			}
		
		
		private: 
			/// \brief Pointer to the model.
			physics::ModelPtr model;

			/// \brief Pointer to the joint.
			physics::Joint_V joints;

			/// \brief A node use for ROS transport
			std::unique_ptr<ros::NodeHandle> rosNode;

			/// \brief A ROS subscriber
			ros::Subscriber rosSubStart;
			ros::Subscriber rosSubStop;
			ros::Subscriber rosSubGaitSel;

			/// \brief A ROS callbackqueue that helps process messages
			ros::CallbackQueue rosQueue;

			// \brief The Gait
			Gait* gait;
			// \brief Gait-specific subscribers:
			std::vector<ros::Subscriber> gaitSubscribers;

			/// \brief A thread the keeps running the rosQueue
			std::thread rosQueueThread;

			// \brief Pointer to the update event connection
	    		event::ConnectionPtr updateConnection;

			// Timing
			ros::Time last_update_sim_time_ros_;

			// Movement Control
			uint8_t isRunning;
	};

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(SimpleGaitPlugin)
}
#endif
