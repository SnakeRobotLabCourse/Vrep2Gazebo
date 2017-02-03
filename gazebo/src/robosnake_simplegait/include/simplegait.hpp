#ifndef SIMPLEGAIT_HPP
#define SIMPLEGAIT_HPP
#include "gait.hpp"

namespace gazebo 
	{
	class SimpleGait : public Gait {
		public: 
			SimpleGait();

			double getAngle(double dt, int joint);
	};
}

#endif
