#include "ros/ros.h"
#include "math.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

class GPS_Odometer{
private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;

	double lat_r, lon_r, alt_r;
	double lat = 0.0, lon = 0.0, alt = 0.0;
	double x, y, z;

	void publish_message(){
		nav_msgs::Odometry _msg;
		_msg.header.frame_id = "odom";
		_msg.child_frame_id = "base_link";
		_msg.pose.pose.position.x = this.x;
		_msg.pose.pose.position.y = this.y;
		_msg.pose.pose.position.z = this.z;
		this.pub.publish(_msg);
		ROS_INFO("GPS odometry has been published.");
	}

	void getGPS(const sensor_msgs::NavSatFix::ConstPtr & _msg){ // TODO usare come callback nel subscriber
		this.lat = _msg->latitude;
		this.lon = _msg->longitude;
		this.alt = _msg->altitude;
		gps_to_odom();
	}

	void gps_to_odom(){ // da chiamare nel publisher
		double X, Y, Z, X_r, Y_r, Z_r, N, N_r, e2;
		e2 = 1 - 6356752^2/6378137^2;
		N_r = 6378137 / (sqrt(1 - e2 * (sin(lat_r))^2));
		N = 6378137 / (sqrt(1 - e2 * (sin(lat))^2));

		// from GPS to ECEF
		X_r = (N_r + alt_r) * cos(lat_r) * cos(lon_r);
		X = (N + alt) * cos(lat) * cos(lon);
		Y_r = (N_r + alt_r) * cos(lat_r) * sin(lon_r);
		Y = (N + alt) * cos(lat) * sin(lon);
		Z_r = (N_r * (1 - e2) + alt_r) * sin(lat_r);
		Z = (N * (1 - e2) + alt) * sin(lat);

		// TODO controllare se i valori sono in radianti o gradi
		// phi = lat, lambda = lon, h = altitude
		// from ECEF to ENU
		double matrix[3][3] = {
			{-sin(lon_r), cos(lon_r), 0},
			{-sin(lat_r) * cos(lon_r), -sin(lat_r) * sin(lon_r), cos(lat_r)},
			{cos(lat_r) * cos(lon_r), cos(lat_r) * sin(lon_r), sin(lat_r)}
		};
		double column[3] = {X - X_r, Y - Y_r, Z - Z_r};
		double c[3] = {0, 0, 0};
		for(int i = 0; i < 3; i++){
			for(int j = 0; i < 3; j++){
				c[i] += matrix[i][j] * column[j];
			}
		}
		this.x = c[0];
		this.y = c[1];
		this.z = c[2];

		/*
		TODO
		After computing the car position in ENU you can use consecutive poses
		to estimate the robot heading (if you want you can also add a smoothing filter)
		*/
		
		publish_message();
	}

public:
	void init(){
		ros::param::get("lat_r", this.lat_r);
		ros::param::get("lon_r", this.lon_r);
		ros::param::get("alt_r", this.alt_r);
		pub = n.advertise<nav_msgs::Odometry>("gps_odom", 1000);
		ROS_INFO("gps_odometer's pub is now started.");
		ros::spin();
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "gps_odometer");
	GPS_Odometer gps_odometer;
	gps_odometer.init();
	ROS_INFO("gps_odometer has been initialized.")
	return 0;
}