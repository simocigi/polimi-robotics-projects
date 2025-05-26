#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class Converter{
private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	tf::TransformBroadcaster tf_br;
	tf::Transform tf_tr;
	tf::Quaternion tf_q;
	geometry_msgs::Quaternion geom_q;

	void convert(const nav_msgs::Odometry::ConstPtr &msg){

	}
public:
	void init(){
		sub = n.subscribe("/odometry", 10, &Converter::convert, this);
		ros::spin();
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "mapping");
	Converter converter;
	converter.init();
	return 0;
}