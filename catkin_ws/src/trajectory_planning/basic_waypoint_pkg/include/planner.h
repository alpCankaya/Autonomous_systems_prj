#ifndef BASIC_WAYPOINT_PKG_PLANNER_H
#define BASIC_WAYPOINT_PKG_PLANNER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <XmlRpcValue.h>
#include <tf/transform_datatypes.h>

class BasicPlanner {
public:
    BasicPlanner(ros::NodeHandle& nh);

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    void goalPositionCallback(const geometry_msgs::Point::ConstPtr& goal);
    void setMaxSpeed(double max_v);
    bool planTrajectory(const Eigen::VectorXd& goal_pos,
                        const Eigen::VectorXd& goal_vel,
                        mav_trajectory_generation::Trajectory* trajectory);
    bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);
    void publishFallbackTrajectory();
    bool isNearTriggerPoint();
    void run();

private:
    ros::NodeHandle& nh_;
    ros::Publisher pub_markers_;
    ros::Publisher pub_trajectory_;
    ros::Publisher pub_mode_switch_;
    ros::Publisher fallback_pub_;
    ros::Publisher new_goal_flag_pub_;
    

    ros::Subscriber sub_odom_;
    ros::Subscriber sub_goal_position_;

    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;

    double max_v_; // m/s
    double max_a_; // m/s^2
    double max_ang_v_;
    double max_ang_a_;

    bool mode_switched_;
    bool new_goal_received_;
    Eigen::Vector3d goal_position_;
    Eigen::Vector3d goal_velocity_;

    std::string goal_position_topic_;
    std::string mode_switch_topic_;

    Eigen::Vector3d trigger_point_;
    double trigger_distance_threshold_;
};

#endif // BASIC_WAYPOINT_PKG_PLANNER_H

