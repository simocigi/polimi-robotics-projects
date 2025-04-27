#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class GPS_Odometer{
private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;
	tf::TransformBroadcaster tf_br;
	tf::Transform tf_tr;
	tf::Quaternion q;

	double lat_r, lon_r, alt_r;
	double lat = 0.0, lon = 0.0, alt = 0.0;
	double lat_prev = 0.0, lon_prev = 0.0;
	double x, y, z, yaw;
	double x_prev = 0.0, y_prev = 0.0;

	//bool ref_set = false; // TODO da rimuovere una volta settati i parametri nel launch

	void getGPS(const sensor_msgs::NavSatFix::ConstPtr & _msg){
		lat = _msg->latitude * M_PI/180;
		lon = _msg->longitude * M_PI/180;
		alt = _msg->altitude;

		// TODO da rimuovere perché i parametri di riferimento sono da settare manualmente nel launch
		/*if (!ref_set) {
			lat_r = lat;
			lon_r = lon;
			alt_r = alt;
			ref_set = true;
		}*/

		gps_to_odom();
		compute_yaw();
		update_prev();
		publish_message();
		publish_tf();
	}

	void gps_to_odom(){ // da chiamare nel publisher
		double X, Y, Z, X_r, Y_r, Z_r, N, N_r;
		const double e2 = 1 - pow(6356752, 2)/pow(6378137, 2);

		// from GPS to ECEF - reference position
		N_r = 6378137 / (sqrt(1 - e2 * (pow(sin(lat_r), 2))));
		X_r = (N_r + alt_r) * cos(lat_r) * cos(lon_r);
		Y_r = (N_r + alt_r) * cos(lat_r) * sin(lon_r);
		Z_r = (N_r * (1 - e2) + alt_r) * sin(lat_r);

		// from GPS to ECEF - robot position
		N = 6378137 / (sqrt(1 - e2 * pow(sin(lat), 2)));
		X = (N + alt) * cos(lat) * cos(lon);
		Y = (N + alt) * cos(lat) * sin(lon);
		Z = (N * (1 - e2) + alt) * sin(lat);

		// from ECEF to ENU
		double matrix[3][3] = {
			{-sin(lon_r), cos(lon_r), 0},
			{-sin(lat_r) * cos(lon_r), -sin(lat_r) * sin(lon_r), cos(lat_r)},
			{cos(lat_r) * cos(lon_r), cos(lat_r) * sin(lon_r), sin(lat_r)}
		};
		double column[3] = {X - X_r, Y - Y_r, Z - Z_r};
		double c[3] = {0, 0, 0};
		for(int i = 0; i < 3; i++){
			for(int j = 0; j < 3; j++){
				c[i] += matrix[i][j] * column[j];
			}
		}
		x = c[0];
		y = c[1];
		z = c[2];
	}

	void compute_yaw(){
		double dx = x - x_prev;
		double dy = y - y_prev;
		double dist = sqrt(pow(dx, 2) + pow(dy, 2));
		yaw = dist < 0.05 ? 90 : atan2(dy, dx) * 180 / M_PI;
	}

	void update_prev(){
		lat_prev = lat;
		lon_prev = lon;
		x_prev = x;
		y_prev = y;
	}

	void publish_message(){
		nav_msgs::Odometry msg;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "start";
		msg.child_frame_id = "gps_odom";
		msg.pose.pose.position.x = x;
		msg.pose.pose.position.y = y;
		msg.pose.pose.position.z = z;
		msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		this->pub.publish(msg);
		ROS_INFO("GPS odometry has been published.");
	}

	void publish_tf(){
		tf_tr.setOrigin(tf::Vector3(x, y, z));
		q.setRPY(0, 0, yaw);   // Only yaw (2D scenario)
		tf_tr.setRotation(q);
		this->tf_br.sendTransform(tf::StampedTransform(tf_tr, ros::Time::now(), "start", "gps_odom"));
		ROS_INFO("Published GPS tf. Position: (%.2f, %.2f, %.2f), Orientation: %.2f", x, y, z, yaw);
	}

public:
	void init(){
		ros::param::get("lat_r", lat_r);
		ros::param::get("lon_r", lon_r);
		ros::param::get("alt_r", alt_r);
		pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1000);
		sub = n.subscribe("/swiftnav/front/gps_pose", 10, &GPS_Odometer::getGPS, this);
		ROS_INFO("gps_odometer's pub and sub are now started.");
		ros::spin();
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "gps_odometer");
	GPS_Odometer gps_odometer;
	gps_odometer.init();
	ROS_INFO("gps_odometer has been initialized.");
	return 0;
}
