#include <math.h>
#include "gait.hpp"
	
using namespace gazebo;

double Gait::toRad(double deg){
	return deg * M_PI / 180;
}
