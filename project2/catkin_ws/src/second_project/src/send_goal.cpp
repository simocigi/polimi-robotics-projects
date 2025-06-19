#include <ros/ros.h>
#include <math.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

class Send_Goal{
private:
    ros::NodeHandle n;
    std::string goal_path;

    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseActionClient;

    void send(double x, double y, double theta){
        MoveBaseActionClient client("move_base", true);
        client.waitForServer();
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta * M_PI/180);
        goal.target_pose.pose.orientation = q;
        client.sendGoal(goal);
        client.waitForResult();
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Goal reached: %f, %f, %f°", x, y, theta);
        else
            ROS_ERROR("Goal aborted: %f, %f, %f°", x, y, theta);
    }

public:
    void init() {
        ros::param::get("csv", goal_path);
        std::fstream file(goal_path);
        if(!file.is_open())
            ROS_ERROR("Couldn't open the csv file!");
        std::string line;
        while(std::getline(file, line)){
            std::vector<std::string> XY0;
            std::stringstream ss(line);
            std::string cell;
            while(std::getline(ss, cell, ','))
                XY0.push_back(cell);
            send(std::stod(XY0.at(0)), std::stod(XY0.at(1)), std::stod(XY0.at(2)));
        }
        file.close();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "send_goal");
    Send_Goal send_goal;
    send_goal.init();
    return 0;
}
