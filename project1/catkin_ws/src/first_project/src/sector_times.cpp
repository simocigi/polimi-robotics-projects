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

    double lat = 0.0, lon = 0.0;
    double steer = 0.0, speed = 0.0;
    double total_time = 0.0, total_distance = 0.0;
    int sector = 1, sector_prev = 1;


    void set_parameters(const geometry_msgs::PointStampedConstPtr & msg1, const sensor_msgs::NavSatFix::ConstPtr & msg2){
        // steer = msg1->point.x;
        speed = msg1->point.y / 3.6;
        lat = msg2->latitude;
        lon = msg2->longitude;
        ROS_INFO ("Received two messages: (%f) and (%f,%f)", speed, lat, lon);
        now = ros::Time::now();
        double dt = (now - last_time).toSec();
        total_time += dt;
        //ROS_INFO("\nlast_time = %f\nnow = %f\ndt = %f\ntotal_time = %f", last_time.toSec(), now.toSec(), dt, total_time);
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
        const double FIRST_SECTOR_LAT = 45.61553154189557;
        const double FIRST_SECTOR_LON = 9.280749006741264;
        const double SECOND_SECTOR_LAT = 45.630096889310636;
        const double SECOND_SECTOR_LON = 9.290098543548453;
        const double THIRD_SECTOR_LAT = 45.62342108025827;
        const double THIRD_SECTOR_LON = 9.286983873232417;
        const double FIRST_THIRD_TRESHOLD_LON = 9.282426505863883;
        const double FIRST_SECOND_TRESHOLD_LAT = 45.62884802683569;

        if(lat == 0 || lon == 0) // if GPS signal is not available
            return sector_prev;
        if(lat <= FIRST_SECTOR_LAT) // yellow bottom curve
            return 3;
        else if(lat >= SECOND_SECTOR_LAT) // blue upper curve
            return 2;
        else if(lon >= SECOND_SECTOR_LON) // blue rightmost part
            return 2;
        else if(lat <= THIRD_SECTOR_LAT && lon >= FIRST_THIRD_TRESHOLD_LON) // yellow part succeeding the blue sector
            return 3;
        else if(lat <= THIRD_SECTOR_LAT && lon < FIRST_THIRD_TRESHOLD_LON) // red part succeeding the yellow sector
            return 1;
        else if(lon <= THIRD_SECTOR_LON) // red upper part
            return 1;
        else if(lat > FIRST_SECOND_TRESHOLD_LAT) // small final part of red sector preceding blue sector
            return 1;
        else
            return 2;
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
            ROS_INFO("\nSector: %d\nTime: %f s\nMean speed: %f m/s", sector, total_time, speed_average);
        }
    }

public:
	void init(){
        reset();
        pub = n.advertise<first_project::Sector_times>("/sector_times", 1000);
        /*do{
            last_time = ros::Time::now();
        }while(!last_time.isValid());*/
        ros::Time::waitForValid();
        last_time = ros::Time::now();
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
