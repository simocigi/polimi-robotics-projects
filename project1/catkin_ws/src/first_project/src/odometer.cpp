#include "ros/ros.h"


class Odometer{
private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;
public:
	void init(){
		
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "odometer");
	Odometer odometer;
	odometer.init();
	return 0;
}