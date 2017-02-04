#ifndef GAIT_HPP
#define GAIT_HPP

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

			void setAmplitude(std::vector<double> a_v, std::vector<double> a_h);
			void setPhase(std::vector<double> p_v, std::vector<double> p_h);
	};
}

#endif
