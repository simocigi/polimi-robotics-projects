#include <ros/ros.h>
#include <math.h>
#include <limits>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class Mapping{
private:
	ros::NodeHandle n;
	ros::Publisher pub;

    tf2::Quaternion q_front, q_back;
    tf2::Vector3    t_front, t_back;

	const float ROBOT_LENGTH = 0.54f, ROBOT_WIDTH = 0.4f, FILTER = 0.35f;
    const double angle_min = - 3.14, angle_max = 3.14, angle_increment = 0.005817;
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
        msg.intensities.assign(N, 0.0f);
        auto insert_scan = [&](const sensor_msgs::LaserScanConstPtr &scan, const tf2::Quaternion &q_s, const tf2::Vector3 &t_s){
            for (size_t i = 0; i < scan->ranges.size(); ++i){
                float r = scan->ranges[i];
                if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max)
                    continue;
                double angle_sensor = scan->angle_min + i * scan->angle_increment;
                double x_s = r * cos(angle_sensor);
                double y_s = r * sin(angle_sensor);
                double z_s = 0.0;
                tf2::Vector3 p_local(x_s, y_s, z_s);
                tf2::Matrix3x3 R(q_s);
                tf2::Vector3 p_base = R * p_local + t_s;
                double x_b = p_base.x();
                double y_b = p_base.y();
                double dist = std::hypot(x_b, y_b);
                double theta = std::atan2(y_b, x_b);
                if (dist < FILTER)
                    continue;
                int idx = static_cast<int>(std::round((theta - angle_min) / angle_increment));
                if (idx >= 0 && idx < N){
                    msg.ranges[idx] = std::min(msg.ranges[idx], static_cast<float>(dist));
                    if (i < scan->intensities.size())
                        msg.intensities[idx] = scan->intensities[i];
                }
            }
        };

        insert_scan(front, q_front, t_front);
        insert_scan(back,  q_back,  t_back);

        pub.publish(msg);
    }

public:
	void init(){
        pub = n.advertise<sensor_msgs::LaserScan>("/scan", 1000);
        q_front.setX(1.0);
        q_front.setY(0.0);
        q_front.setZ(0.0);
        q_front.setW(0.0);
        t_front.setX(0.3);
        t_front.setY(0.0);
        t_front.setZ(-0.115);
        q_back.setX(0.0);
        q_back.setY(1.0);
        q_back.setZ(0.0);
        q_back.setW(0.0);
        t_back.setX(-0.3);
        t_back.setY(0.0);
        t_back.setZ(-0.115);
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