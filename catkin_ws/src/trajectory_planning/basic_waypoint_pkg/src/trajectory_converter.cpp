#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <std_msgs/Bool.h>

ros::Publisher pub;
bool new_goal_flag = false; // default false

// Callback for /desired_trajectory
void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& traj_msg) {
    if (!new_goal_flag) {
        ROS_WARN_THROTTLE(5.0, "No new goal - ignoring incoming trajectory");
        return;
    }

    // If flag is true, publish the points
    for (const auto& point : traj_msg->points) {
        pub.publish(point);
    }
}

// Callback for /new_goal_flag
void newGoalFlagCallback(const std_msgs::Bool::ConstPtr& msg) {
    new_goal_flag = msg->data;
    if (new_goal_flag) {
        ROS_INFO("Trajectory Converter: new goal flag is TRUE.");
    } else {
        ROS_INFO("Trajectory Converter: new goal flag is FALSE.");
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_converter");
    ros::NodeHandle nh;

    // Publisher for MultiDOFJointTrajectoryPoint
    pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 10);

    // Subscriber to the original MultiDOFJointTrajectory
    ros::Subscriber sub = nh.subscribe("/desired_trajectory", 10, trajectoryCallback);

    // NEW: Subscribe to the bool flag
    ros::Subscriber flag_sub = nh.subscribe("/new_goal_flag", 10, newGoalFlagCallback);

    ROS_INFO("Trajectory Converter Node Started.");
    ros::spin();

    return 0;
}

