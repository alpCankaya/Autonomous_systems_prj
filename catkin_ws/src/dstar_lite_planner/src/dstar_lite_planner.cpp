#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <limits>
#include <cmath>
#include <vector>

//-----------------------------------------------------
// 1) Simple placeholder D* or BFS-like function
//-----------------------------------------------------
//
// Here we pretend to do a D* search or BFS in the map.
// For demonstration, we pick the closest free cell
// that is far from the robot's current pose, or
// we can pick a random free frontier cell, etc.
//
// This function returns the next best (x,y) in world coords.
bool runDStarSearch(
    const nav_msgs::OccupancyGrid& map,
    double current_x, double current_y,
    double& goal_x, double& goal_y)
{
    // Placeholder: search for a free cell that is “interesting”.
    // In a real D* or frontier-based approach, you’d fill in the logic
    // to do a real search for the best frontier or path to an unexplored cell.
    // For demonstration, we just pick an arbitrary cell 3m forward.

    // For example, just set the next goal to (x+3, y) in world coordinates:
    goal_x = current_x + 3.0;
    goal_y = current_y;

    // Return true if we found a valid next goal
    return true;
}

//-----------------------------------------------------
// 2) Main class for a D* Node
//-----------------------------------------------------
class DStarExplorer
{
public:
    DStarExplorer()
    : map_received_(false), odom_received_(false)
    {
        ros::NodeHandle nh, pnh("~");

        // Subscribers
        map_sub_  = nh.subscribe("/projected_map", 1, &DStarExplorer::mapCallback, this);
        odom_sub_ = nh.subscribe("/current_state_est", 1, &DStarExplorer::odomCallback, this);

        // Publisher to pass the next goal position to the planner
        // planner.cc expects /goal_position
        goal_pub_ = nh.advertise<geometry_msgs::Point>("/goal_position", 1);

        // Timer to run the D* planning at a given frequency
        double plan_rate = 1.0; // once per second
        pnh.getParam("plan_rate", plan_rate);
        plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_rate),
                                     &DStarExplorer::planTimerCallback,
                                     this);

        ROS_INFO("DStarExplorer node started!");
    }

private:
    // ROS handles
    ros::Subscriber map_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher  goal_pub_;
    ros::Timer      plan_timer_;

    // Data for map & current odom
    nav_msgs::OccupancyGrid map_;
    bool map_received_;

    double current_x_, current_y_;
    bool odom_received_;

    //-----------------------------------------------------
    // Callback: map
    //-----------------------------------------------------
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        map_ = *msg;
        map_received_ = true;
        ROS_INFO_ONCE("Map received.");
    }

    //-----------------------------------------------------
    // Callback: current UAV pose from Odometry
    //-----------------------------------------------------
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        odom_received_ = true;
    }

    //-----------------------------------------------------
    // Timer callback that attempts to run D* & publish next goal
    //-----------------------------------------------------
    void planTimerCallback(const ros::TimerEvent&)
    {
        if (!map_received_ || !odom_received_)
        {
            ROS_WARN_THROTTLE(5.0, "DStarExplorer waiting for map & odom data...");
            return;
        }

        // 1) Run D* (or BFS, or frontier detection) to find next best goal
        double goal_x, goal_y;
        bool found_goal = runDStarSearch(map_, current_x_, current_y_, goal_x, goal_y);
        if(!found_goal)
        {
            ROS_WARN("DStar: No valid goal found. Not publishing anything.");
            return;
        }

        // 2) Publish the next goal for the planner
        geometry_msgs::Point goal_msg;
        goal_msg.x = goal_x;
        goal_msg.y = goal_y;
        goal_msg.z = 0.0;

        goal_pub_.publish(goal_msg);

        ROS_INFO("DStarExplorer => Next goal: (%.2f, %.2f)", goal_x, goal_y);
    }
};

//-----------------------------------------------------
// 3) main()
//-----------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dstar_explorer");
    DStarExplorer explorer;
    ros::spin();
    return 0;
}
