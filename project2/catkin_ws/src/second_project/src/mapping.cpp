#include <ros/ros.h>
#include <math.h>
#include <limits>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

class Mapping{
private:
	ros::NodeHandle n;
	ros::Publisher pub;

	const float ROBOT_LENGTH = 0.54f, ROBOT_WIDTH = 0.4f;
    const double angle_min = - M_PI, angle_max = M_PI, angle_increment = 0.00581718236207962;
    const int N = ceil((angle_max - angle_min) / angle_increment);

	void merge_scan(const sensor_msgs::LaserScanConstPtr &front, const sensor_msgs::LaserScanConstPtr &back){
        sensor_msgs::LaserScan msg;
        msg.header.stamp = front->header.stamp;
        msg.header.frame_id = "base_link";
        msg.angle_min = angle_min;
        msg.angle_max = angle_max;
        msg.angle_increment = angle_increment;
        msg.time_increment = front->time_increment;
        msg.scan_time = front->scan_time;
        msg.range_min = front->range_min;
        msg.range_max = front->range_max;

        msg.ranges.assign(N, std::numeric_limits<float>::infinity());
        //msg.intensities.assign(N, 0.0f);

        for(int i = 0; i < back->ranges.size(); i++){
            float r = back->ranges[i];
            if (r >= back->range_min && r <= back->range_max)
                msg.ranges[i] = r < 0.3f ? std::numeric_limits<float>::infinity() : r;
        }
        /*float angle = front->angle_min;
        for (size_t i = 0; i < front->ranges.size(); ++i, angle += front->angle_increment){
            int idx = static_cast<int>(std::round((angle - angle_min) / angle_increment));
            if (idx < 0 || idx >= N)
                continue;
            float r = front->ranges[i];
            if (r >= front->range_min && r <= front->range_max){
                // msg.ranges[idx] = (r < 0.2f) ? std::numeric_limits<float>::infinity() : r;
                msg.ranges[idx] = r;
                if (i < front->intensities.size())
                    msg.intensities[idx] = front->intensities[i];
            }
        }*/

        /*angle = back->angle_min;
        for (size_t i = 0; i < back->ranges.size(); ++i, angle += back->angle_increment){
            float a = angle;
            while (a >  M_PI) a -= 2 * M_PI;
            while (a < -M_PI) a += 2 * M_PI;
            int idx = static_cast<int>(std::round((a - angle_min) / angle_increment));
            if (idx < 0 || idx >= N)
                continue;
            float r = back->ranges[i];
            if (r >= back->range_min && r <= back->range_max){
                // msg.ranges[idx] = (r < 0.2f) ? std::numeric_limits<float>::infinity() : r;
                msg.ranges[idx] = r;
                if (i < back->intensities.size())
                    msg.intensities[idx] = back->intensities[i];
            }
        }*/

        pub.publish(msg);
    }

public:
	void init(){
        pub = n.advertise<sensor_msgs::LaserScan>("/scan", 1000);
		message_filters::Subscriber<sensor_msgs::LaserScan> sub1(n, "/scan_front", 10);
		message_filters::Subscriber<sensor_msgs::LaserScan> sub2(n, "/scan_back", 10);
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
		sync.registerCallback(boost::bind(&Mapping::merge_scan, this, _1, _2));
		ros::spin();
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "mapping");
	Mapping mapping;
	mapping.init();
	return 0;
}