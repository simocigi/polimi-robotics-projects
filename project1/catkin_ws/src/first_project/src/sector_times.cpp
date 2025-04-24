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
    ros::Time now, last_time;

    double lat = 0.0, lon = 0.0, lat_prev, lon_prev;
    double steer = 0.0, speed = 0.0;
    double total_time, total_distance;
    int sector = 0, sector_prev = 0;


    void set_parameters(const geometry_msgs::PointStampedConstPtr & msg1, const sensor_msgs::NavSatFix::ConstPtr & msg2){
        // steer = msg1->point.x;
        speed = msg1->point.y / 3.6;
        lat = msg2->latitude;
        lon = msg2->longitude;
        ROS_INFO ("Received two messages: (%f) and (%f,%f)", speed, lat, lon);
        now = ros::Time::now();
        double dt = (now - last_time).toSec();
        total_time += dt;
        last_time = now;
        sector = localize_sector();
        if(sector == sector_prev)
            publish(dt);
        else{
            reset();
            publish(dt);
        }
    }

    int localize_sector(){
        return 0;
    }

    void reset(){
        total_distance = 0;
        total_time = 0;
        last_time = ros::Time::now();
    }

    void publish(double dt){
        total_distance += speed * dt;
        sector_prev = sector;
        if(total_time != 0){
            double speed_average = total_distance / total_time;
            first_project::Sector_times msg;
            msg.current_sector = sector;
            msg.current_sector_time = total_time;
            msg.current_sector_mean_speed = speed_average;
            pub.publish(msg);
            ROS_INFO("sector times' message has been published.");
            ROS_INFO("Sector: %d\nTime: %f s\nMean speed: %f m/s", sector, total_time, speed_average);
        }
    }

public:
	void init(){
        pub = n.advertise<first_project::Sector_times>("/sector_times", 1000);
        do{
            last_time = ros::Time::now();
        }while(!last_time.isValid());
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
