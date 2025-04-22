#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <first_project/Sector_times.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

class Sector_Times{
private:
	ros::NodeHandle n;
	ros::Publisher pub;

    double lat = 0.0, lon = 0.0;
    double steer = 0.0, speed = 0.0;

    void set_parameters(const geometry_msgs::PointStampedConstPtr & msg1, const sensor_msgs::NavSatFix::ConstPtr & msg2){
        steer = msg1->point.x;
        speed = msg1->point.y;
        lat = msg2->latitude;
        lon = msg2->longitude;
        ROS_INFO ("Received two messages: (%f,%f) and (%f,%f)", steer, speed, lat, lon);
        localize_sector();
    }

    void localize_sector(){

    }

    void timer(){
        
    }

    void compute_mean_speed(){

    }

    void publish(){

    }

public:
	void init(){
        pub = n.advertise<first_project::Sector_times>("/sector_times", 1000);
        message_filters::Subscriber<geometry_msgs::PointStamped> sub1(n, "/speedsteer", 1);
        message_filters::Subscriber<sensor_msgs::NavSatFix> sub2(n, "/swiftnav/front/gps_pose", 1);
        //message_filters::TimeSynchronizer<geometry_msgs::PointStamped, sensor_msgs::NavSatFix> sync(sub1, sub2, 10);
        //sync.registerCallback(boost::bind(&Sector_Times::set_parameters, this, _1, _2));
        typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, sensor_msgs::NavSatFix> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
        sync.registerCallback(boost::bind(&Sector_Times::set_parameters, this, _1, _2));
        ROS_INFO("sector_times's pub and subs are now started.");
        ros::spin();
    }
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "sector_times");
	Sector_Times sector_times;
	sector_times.init();
	return 0;
}