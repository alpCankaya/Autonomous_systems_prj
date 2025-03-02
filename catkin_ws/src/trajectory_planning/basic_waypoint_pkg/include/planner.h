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
#include <XmlRpcValue.h>
#include <tf/transform_datatypes.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

class BasicPlanner {
public:
    // Constructor
    BasicPlanner(ros::NodeHandle& nh);

    // Callback for odometry updates
    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);

    // Callback for receiving new goal positions
    void goalPositionCallback(const geometry_msgs::Point::ConstPtr& goal);

    // Set maximum speed parameter
    void setMaxSpeed(const double max_v);

    // Plan a trajectory from current state to the goal
    bool planTrajectory(const Eigen::VectorXd& goal_pos,
                        const Eigen::VectorXd& goal_vel,
                        mav_trajectory_generation::Trajectory* trajectory);

    // Publish the planned trajectory for visualization and execution
    bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);
    
    // Publish a fallback trajectory (e.g., a static pose) when no valid goal is available
    void publishFallbackTrajectory();

    // Check if the UAV is near the trigger point
    bool isNearTriggerPoint();

    // Main run loop
    void run();

private:
    // Publishers
    ros::Publisher pub_markers_;
    ros::Publisher pub_trajectory_;
    ros::Publisher pub_mode_switch_;
    ros::Publisher fallback_pub_;

    // Subscribers
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_goal_position_;

    // Node handle
    ros::NodeHandle& nh_;

    // State variables
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;
    
    // Maximum speed and acceleration parameters
    double max_v_; // m/s
    double max_a_; // m/s^2
    double max_ang_v_;
    double max_ang_a_;

    // Mode flags and goal state
    bool mode_switched_;
    bool new_goal_received_;
    Eigen::Vector3d goal_position_;
    Eigen::Vector3d goal_velocity_;
    
    // Topic names
    std::string goal_position_topic_;
    std::string mode_switch_topic_;
    
    // Trigger point and its threshold distance for switching modes
    Eigen::Vector3d trigger_point_;
    double trigger_distance_threshold_;
};

#endif // BASIC_WAYPOINT_PKG_PLANNER_H

