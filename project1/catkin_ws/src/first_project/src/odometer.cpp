/*#include "ros/ros.h"
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
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class OdomPub {
private:
    // ROS node handle
    ros::NodeHandle nh_;

    // Subscribers for the "/chatter" and "/chatter2" topics
    ros::Subscriber speedsteer_sub_;

    // Publisher for the "/rechatter" topic
    ros::Publisher odometry_pub_;
    ros::Time last_time_;

    tf::TransformBroadcaster tf_broadcaster_;

    // robot state variables
    double x_, y_, yaw_;

    const double WHEELBASE = 1.765; // meters

    const int STEERING_FACTOR = 32;

public:
    // Constructor: sets up subscribers, publisher, and timer
    OdomPub() {
        // Subscribe to topics with a queue size of 1
        speedsteer_sub_ = nh_.subscribe("/speedsteer", 1, &OdomPub::callback, this);

	odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);

	last_time_ = ros::Time::now();
    }

    // Callback function for the "/chatter" topic
    void callback(const geometry_msgs::PointStamped::ConstPtr& msg) {

        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;

        // Conversions
        double steer_deg = msg->point.x;
        double speed_kmh = msg->point.y;
        double steer_rad = steer_deg/STEERING_FACTOR * M_PI / 180.0;
        double speed = speed_kmh / 3.6; // m/s

        // Bicycle model
        double omega = speed / WHEELBASE * tan(steer_rad);
        yaw_ += omega * dt;
        x_ += speed * cos(yaw_) * dt;
        y_ += speed * sin(yaw_) * dt;

        // TF
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x_, y_, 0.0));

	tf::Quaternion q;
	q.setRPY(0, 0, yaw_);
	transform.setRotation(q);

	tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "vehicle"));

        // compiling odometry message
	geometry_msgs::Quaternion theta_quaternions = tf::createQuaternionMsgFromYaw(yaw_);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "vehicle";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation = theta_quaternions;

        odom.twist.twist.linear.x = speed;
        odom.twist.twist.angular.z = omega;
        odometry_pub_.publish(odom);

	ROS_INFO_STREAM("x: " << x_ << " y: " << y_ << " yaw: " << yaw_);
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_publisher");
    OdomPub node;
    ros::spin();
    return 0;
}

