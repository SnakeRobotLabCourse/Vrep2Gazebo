#ifndef _GAIT_PLUGIN_HH_
#define _GAIT_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "gait.hpp"
#include "simplegait.hpp"

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
	class SimpleGaitPlugin : public ModelPlugin
	{
		/// \brief Constructor
		public: SimpleGaitPlugin() {}

		/// \brief The load function is called by Gazebo when the plugin is
		/// inserted into simulation
		/// \param[in] _model A pointer to the model that this plugin is
		/// attached to.
		/// \param[in] _sdf A pointer to the plugin's SDF element.
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			
			this->model = _model;
			this->joints = _model->GetJoints();

			// Setup PID controllers
			for(unsigned i = 0; i < this->joints.size(); i++){
				this->model->GetJointController()->SetVelocityPID(this->joints[i]->GetScopedName(), common::PID(33, 1.2, 0));
			}
			
			// Initialize ros, if it has not already bee initialized.
			if (!ros::isInitialized())
			{
			  int argc = 0;
			  char **argv = NULL;
			  ros::init(argc, argv, "gazebo_client",
			      ros::init_options::NoSigintHandler);
			}

			// Create our ROS node. This acts in a similar manner to
			// the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so =
			  ros::SubscribeOptions::create<std_msgs::Float32>(
			      "/" + this->model->GetName() + "/simplegait_cmd",
			      1,
			      boost::bind(&SimpleGaitPlugin::OnRosMsg, this, _1),
			      ros::VoidPtr(), &this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);

			// Spin up the queue helper thread.
			this->rosQueueThread =
			  std::thread(std::bind(&SimpleGaitPlugin::QueueThread, this));

			// Bind Gait Polymorphic to SimpleGait
			SimpleGait* gait = new SimpleGait();
			this->gaitPtr = gait;

			// Listen to the update event. This event is broadcast every
      			// simulation iteration.
      			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          			boost::bind(&SimpleGaitPlugin::OnUpdate, this, _1));

			std::cerr << "\nThe simple gait control plugin is attached to model[" <<
				_model->GetName() << "] with " << this->joints.size() << " joints\n";
			
		}

		// Called by the world update start event
    		public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    		{
			// Get the simulation time and period
			gazebo::common::Time gz_time_now = this->model->GetWorld()->GetSimTime();
			ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
			ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

			// Update joint positions
			for(unsigned i = 0; i < this->joints.size(); i++){
				double pos = this->gaitPtr->getAngle(sim_period.toSec(), i);
				this->model->GetJointController()->SetPositionTarget(this->joints[i]->GetScopedName(), pos);
			}
    		}

		/// \brief Handle an incoming message from ROS
		/// \param[in] _msg A float value that is used to set the velocity
		/// of the Velodyne.
		public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
		{
		  //this->SetVelocity(_msg->data);
			printf("message\n");
		}

		/// \brief ROS helper function that processes messages
		private: void QueueThread()
		{
			static const double timeout = 0.01;
			while (this->rosNode->ok())
			{	
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
			}
		}
		
		/// \brief Pointer to the model.
		private: physics::ModelPtr model;

		/// \brief Pointer to the joint.
		private: physics::Joint_V joints;

		/// \brief A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

		/// \brief A ROS subscriber
		private: ros::Subscriber rosSub;

		/// \brief A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

		// \brief The Gait
		private: Gait* gaitPtr;

		/// \brief A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;

		// \brief Pointer to the update event connection
    		private: event::ConnectionPtr updateConnection;

		// Timing
		ros::Duration control_period_;
		ros::Time last_update_sim_time_ros_;
		ros::Time last_write_sim_time_ros_;
	};

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(SimpleGaitPlugin)
}
#endif
