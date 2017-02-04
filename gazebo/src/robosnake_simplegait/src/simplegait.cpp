#include <vector>
#include <math.h>
#include "gait.hpp"
#include "simplegait.hpp"

using namespace gazebo;

SimpleGait::SimpleGait(){
	speed={-2.0, -2.0};
	a_h = {toRad(20.0), toRad(30.0)};
	a_v = {toRad(20.0), toRad(30.0)};
	p_v = {0.0, 0.0};
	p_h = {0.0, 0.0};
	t = 0.0;
	s = 0.0;
}

double SimpleGait::getAngle(double dt, int joint){
	if(joint == 0){
		t += dt;
		if (t > 20.0){
			s = t - 20.0;
			if (s > 1.0){
				s = 1.0;
			}
		}
	}
	if (joint % 2 == 1){
		return (a_v[0] * (1.0 - s) + a_v[1] * s) * sin(t * (speed[0] * (1.0 - s) + speed[1] * s) 
			+ (joint / 2 + 1)  * (p_v[0] * (1.0 - s) + p_v[1] * s));
	} else {
		return (a_h[0] * (1.0 - s) + a_h[1] * s) * cos(t * (speed[0] * (1.0 - s) + speed[1] * s) 
			+ (joint / 2 + 1) * (p_h[0] * (1.0 - s) + p_h[1] * s));
	}
	
}
