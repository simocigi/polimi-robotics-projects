#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

class Mapping{
private:
	ros::NodeHandle n;
	ros::Publisher pub;

	const float ROBOT_LENGTH = 0.54, ROBOT_WIDTH = 0.4;
    const double angle_min = - M_PI, angle_max = M_PI, angle_increment = 0.0058, range_min = 0, range_max = 100;
    const int N = ceil((angle_max - angle_min) / angle_increment);

	void merge_scan(const sensor_msgs::LaserScanConstPtr &msg1, const sensor_msgs::LaserScanConstPtr &msg2){
        double ranges[N];

    }

    void filter_robot(){

    }
public:
	void init(){
		message_filters::Subscriber<sensor_msgs::LaserScan> sub1(n, "/scan_front", 1);
		message_filters::Subscriber<sensor_msgs::LaserScan> sub2(n, "/scan_back", 1);
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
		sync.registerCallback(boost::bind(&Mapping::merge_scan, this, _1, _2));
		ros::spin();
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "mapping");
	Mapping mapping;
	mapping.init();
	return 0;
}