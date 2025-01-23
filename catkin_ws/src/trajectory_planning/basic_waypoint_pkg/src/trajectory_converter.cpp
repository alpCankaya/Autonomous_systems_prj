#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

ros::Publisher pub;

void trajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& traj_msg) {
    for (const auto& point : traj_msg->points) {
        pub.publish(point);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_converter");
    ros::NodeHandle nh;

    // Publisher for MultiDOFJointTrajectoryPoint
    pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 10);

    // Subscriber to MultiDOFJointTrajectory
    ros::Subscriber sub = nh.subscribe("/desired_trajectory", 10, trajectoryCallback);

    ROS_INFO("Trajectory Converter Node Started.");
    ros::spin();

    return 0;
}
