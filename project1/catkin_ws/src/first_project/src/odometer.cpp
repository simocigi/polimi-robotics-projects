#include "ros/ros.h"
#include "math.h"
#include <nav_msgs/Odometry.h>	
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>

class Odometer{
private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;
	ros::Time last_time;

	const double d = 0.1765; //distance from front to rear wheel
	double x = 0; y = 0; theta = 0; //valori iniziali
	
	void publishOdometry(){
		nav_msgs::Odometry odometry_msg;
		geometry_msgs::Quaternion rotation_q = tf::createQuaternionMsgFromYaw(theta);		
		
		odometry_msg.pose.pose.position.x = x;
		odometry_msg.pose.pose.position.y = y;
		odometry_msg.pose.pose.orientation = rotation_q;
		
		odometryPublisher.publish(odometry_msg);
            	ROS_INFO("odometry message has been published!");
	}
	
	void computeOdometry(const geometry_msgs::PointStamped::constPtr &data){
		double alpha = data->point.x *3.14/32/180; //steering angle [rad]
		double V = data->point.y *3.6; //odometric velocity [m/s]
		double delta_t;

		ros::Time current_time = ros::Time::now();
		delta_t = current_time.toSec() - last_time.toSec();
	
		//Euler Integration	
		x = x + V*cos(theta)*delta_t;
		y = y + V*sin(theta)*delta_t;
		theta = theta + omega*delta_t;
		
		publishOdometry();
		last_time = current_time;

	}
public:
	void init(){
		odometryPublisher = n.advertise<nav_msgs::Odometry>("odom", 1000);	
		
		ROS_INFO("publisher started----------");
		
		do{
			last_time = ros::Time::now();
		} while(!last_time.isValid());
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "odometer");
	Odometer odometer;
	odometer.init();
	return 0;
}
