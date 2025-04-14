#include "ros/ros.h"
#include "math.h"
# include <nav_msgs/Odometry.h>	

class Odometer{
private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;

	double alpha; //steering angle from /speedsteer topic [deg]
	double V; //odometric center velocity from /speedsteer [km/h]

	//output odometry
	nav_msgs::Odometry odometry_msg;

	void Vhdata_odometry(double alpha_star, double V){
		//need alpha in [rad] and V in [m/s] !!

		double alpha = alpha_star/32;
		int d = 1.765; //distance from front to rear wheels [m]

	}
public:
	void init(){
			
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "odometer");
	Odometer odometer;
	odometer.init();
	return 0;
}
