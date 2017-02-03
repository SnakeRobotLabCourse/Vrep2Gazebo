#include <vector>
#include <math.h>
#include "gait.hpp"
#include "simplegait.hpp"

using namespace gazebo;

SimpleGait::SimpleGait(){
	speed={-4, -4};
	a_h = {toRad(20), toRad(30)};
	a_v = {toRad(20), toRad(30)};
	p_v = {0, 0};
	p_h = {0, 0};
	t = 0;
	s = 0;
}

double SimpleGait::getAngle(double dt, int joint){
	t += dt;
	if (t > 20){
		s = t - 20;
		if (s > 1){
			s = 1;
		}
	}

	if (joint % 2 == 1){
		return (a_v[0] * (1 - s) + a_v[1] * s) * sin(t * (speed[0] * (1 - s) + speed[1] * s) 
			+ joint * (p_v[0] * (1 - s) + p_v[2] * s));
	} else {
		return (a_h[0] * (1 - s) + a_h[1] * s) * cos(t * (speed[0] * (1 - s) + speed[1] * s) 
			+ joint * (p_h[0] * (1 - s) + p_h[2] * s));
	}
	
}
