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

/*
#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class GPS_Odometer {
private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
    tf::TransformBroadcaster tf_br;

    // References
    double lat_r, lon_r, alt_r;
    bool ref_set = false;   // Flag for reference point initialization
    double lat_prev, lon_prev, x_prev, y_prev;
    double lat, lon, alt, x, y, z, yaw;

    void publish_message() {
        nav_msgs::Odometry msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";

        msg.pose.pose.position.x = this->x;
        msg.pose.pose.position.y = this->y;
        msg.pose.pose.position.z = this->z;

        msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->yaw);

        this->pub.publish(msg);

        // Broadcast tf transform: odom -> gps
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(this->x, this->y, this->z));
        tf::Quaternion q;
        q.setRPY(0, 0, this->yaw);   // Only yaw (2D scenario)
        transform.setRotation(q);

	this->tf_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

        ROS_INFO("Published GPS odometry. Position: (%.2f, %.2f, %.2f), Orientation: %.2f", x, y, z, yaw);

    }

    void getGPS(const sensor_msgs::NavSatFix::ConstPtr& msg) {
      
        // Convert latitude, longitude to radians
        this->lat = msg->latitude * M_PI / 180.0;
        this->lon = msg->longitude * M_PI / 180.0;
        this->alt = msg->altitude;

        // Set the reference point on the first valid GPS data
        if (!ref_set) {
            lat_r = lat;
            lon_r = lon;
            alt_r = alt;
            ref_set = true;
        }

        gps_to_odom();
   
	compute_yaw();

        // Update previous position
        lat_prev = lat;
        lon_prev = lon;
        x_prev = x;
        y_prev = y;

	publish_message();
    }

    void gps_to_odom() {

        const double a = 6378137.0;   // Semi major axis
        const double b = 6356752.0;   // Semi minor axis
        const double e2 = 1.0 - (b*b)/(a*a);

        // ECEF for reference position
        double N_r = a / sqrt(1 - e2 * pow(sin(lat_r), 2));
        double X_r = (N_r + alt_r) * cos(lat_r) * cos(lon_r);
        double Y_r = (N_r + alt_r) * cos(lat_r) * sin(lon_r);
        double Z_r = (N_r * (1 - e2) + alt_r) * sin(lat_r);

        // ECEF for robot position
        double N = a / sqrt(1 - e2 * pow(sin(lat), 2));
        double X = (N + alt) * cos(lat) * cos(lon);
        double Y = (N + alt) * cos(lat) * sin(lon);
        double Z = (N * (1 - e2) + alt) * sin(lat);

        // ECEF to ENU
        //double dx = X - X_r;
        //double dy = Y - Y_r;
        //double dz = Z - Z_r;

        double matrix[3][3] = {
            {-sin(lon_r),                cos(lon_r),               0},
            {-sin(lat_r)*cos(lon_r), -sin(lat_r)*sin(lon_r), cos(lat_r)},
            { cos(lat_r)*cos(lon_r),  cos(lat_r)*sin(lon_r), sin(lat_r)}
        };

        double c[3] = {0, 0, 0};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                c[i] += matrix[i][j] * (j == 0 ? X-X_r : j == 1 ? Y-Y_r : Z-Z_r);

        this->x = c[0];
        this->y = c[1];
        this->z = c[2];
    }

    void compute_yaw() {
	double dx = x - x_prev;
    	double dy = y - y_prev;
    	double dist = sqrt(dx*dx + dy*dy);

	if (dist < 0.5) this->yaw = 90.0;  //Threshold 
	else this->yaw = atan2(dy, dx) * 180.0 / M_PI;  // Orientation in degrees
	}

public:
    void init() {
        pub = n.advertise<nav_msgs::Odometry>("gps_odom", 1000);
        sub = n.subscribe("/swiftnav/front/gps_pose", 10, &GPS_Odometer::getGPS, this);

        ROS_INFO("GPS odometer node initialized and subscribed.");
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_odometer");
    GPS_Odometer gps_odometer;
    gps_odometer.init();
    return 0;
}
*/
