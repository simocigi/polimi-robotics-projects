#include <ros/ros.h>
#include <math.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class Odometer{
private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;
	ros::Time last_time;
	tf::TransformBroadcaster tf_br;
	tf::Transform tf_tr;
	tf::Quaternion tf_q;
	geometry_msgs::Quaternion geom_q;

	const double wheelbase = 1.765;
	const int steer_factor = 32;
	double x = 0.0, y = 0.0, yaw = 90;

	void compute_odometry(const geometry_msgs::PointStampedConstPtr & _msg){
		double steer = _msg->point.x / steer_factor * M_PI / 180; // rad
		double speed = _msg->point.y / 3.6; // m/s
		ros::Time current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		last_time = current_time;
		double omega = speed / wheelbase * tan(steer);
        // Euler integration
	/*	x += speed * cos(yaw) * dt;
		y += speed * sin(yaw) * dt;
		yaw += omega * dt;
	*/

	//exact integration + runge-kutta
		if(omega<0.01 && omega>-0.01){
			//runge-kutta	
			x += speed * dt * cos(yaw + omega*dt/2);
			y += speed * dt * sin(yaw + omega*dt/2);
			yaw += omega * dt;
		} else{
			//exact approx
			double old_yaw = yaw;
			yaw += omega * dt;
			x += (speed / omega) * (sin(yaw) - sin(old_yaw));
			y -= (speed / omega) * (cos(yaw) - cos(old_yaw));
		}
		
		publish_message(speed, omega);
		publish_tf();
	}

	void publish_message(double speed, double omega){
		nav_msgs::Odometry msg;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "start";
		msg.child_frame_id = "odom";
		msg.pose.pose.position.x = x;
		msg.pose.pose.position.y = y;
		msg.pose.pose.position.z = 0.0;
		msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		msg.twist.twist.linear.x = speed*cos(yaw);
		msg.twist.twist.linear.y = speed*sin(yaw);
		msg.twist.twist.angular.z = omega;
		this->pub.publish(msg);
		ROS_INFO("Published message. Position: (%.2f, %.2f, 0.0), Orientation: %.2f", x, y, yaw);
	}

	void publish_tf(){
		tf_tr.setOrigin(tf::Vector3(x, y, 0.0));
		tf_q.setRPY(0, 0, yaw);
		tf_tr.setRotation(tf_q);
		tf_br.sendTransform(tf::StampedTransform(tf_tr, ros::Time::now(), "start", "odom"));
		ROS_INFO("Published tf. Position: (%.2f, %.2f, 0.0), Orientation: %.2f", x, y, yaw);
	}
	

public:
	void init(){
		pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);
		do{
			last_time = ros::Time::now();
		}while(!last_time.isValid());
        sub = n.subscribe("/speedsteer", 10, &Odometer::compute_odometry, this);
        ROS_INFO("odometer's pub and sub are now started.");
		ros::spin();
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "odometer");
	Odometer odometer;
	odometer.init();
	return 0;
}
