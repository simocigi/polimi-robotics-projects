#include "ros/ros.h"


class Sector_Times{
private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;
public:
	void init(){
		
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "sector_times");
	Sector_Times sector_times;
	sector_times.init();
	return 0;
}