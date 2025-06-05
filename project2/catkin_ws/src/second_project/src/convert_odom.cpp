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

	double x = 0.0, y = 0.0;

	void convert(const nav_msgs::Odometry::ConstPtr &msg){
		x = msg->pose.pose.position.x;
		y = msg->pose.pose.position.y;
        tf_tr.setOrigin(tf::Vector3(x, y, 0.0));
		tf::Quaternion tf_q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w); 		
		tf_tr.setRotation(tf_q);
		tf_br.sendTransform(tf::StampedTransform(tf_tr, msg->header.stamp, "odom", "base_link"));
        //ROS_INFO("TF sent from odom to base_link");
	}
public:
	void init(){
		sub = n.subscribe("/odometry", 10, &Converter::convert, this);
		ros::spin();
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "convert_odom");
	Converter converter;
	converter.init();
	return 0;
}
