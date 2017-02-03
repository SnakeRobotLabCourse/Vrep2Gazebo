#include <vector>
#include <math.h>
#include "gait.hpp"
	
using namespace gazebo;

void Gait::setAmplitude(std::vector<double> a_v, std::vector<double> a_h){
	a_h = {toRad(a_h[0]), toRad(a_h[1])};
	a_v = {toRad(a_v[0]), toRad(a_v[1])};
}

void Gait::setPhase(std::vector<double> p_v, std::vector<double> p_h){
	p_h = {toRad(p_h[0]), toRad(p_h[1])};
	p_v = {toRad(p_v[0]), toRad(p_v[1])};
}

double Gait::toRad(double deg){
	return deg * M_PI / 180;
}
