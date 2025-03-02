#include <planner.h>

BasicPlanner::BasicPlanner(ros::NodeHandle& nh) :
        nh_(nh),
        max_v_(0.2),
        max_a_(0.2),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity()),
        mode_switched_(false),
        new_goal_received_(false),
        goal_velocity_(Eigen::Vector3d::Zero()),
        trigger_distance_threshold_(1.5) {

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  To Do: Load Trajectory Parameters from file
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // In this section, you need to use node handler to get max v and max a params
    //
    // ~~~~ begin solution
    //
    if (!nh_.getParam("/planner/dynamic_params/max_v", max_v_)) {
        ROS_WARN("Param 'max_v' not found; using default %f", max_v_);
    }
    if (!nh_.getParam("/planner/dynamic_params/max_a", max_a_)) {
        ROS_WARN("Param 'max_a' not found; using default %f", max_a_);
    }

    //
    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //                                 end
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Get topic names from parameters or use defaults
    if (!nh_.getParam("/planner/goal_position_topic", goal_position_topic_)) {
        goal_position_topic_ = "/goal_position";
        ROS_WARN("Param 'goal_position_topic' not found; using default: %s", goal_position_topic_.c_str());
    }
    
    if (!nh_.getParam("/planner/mode_switch_topic", mode_switch_topic_)) {
        mode_switch_topic_ = "/mode_switch";
        ROS_WARN("Param 'mode_switch_topic' not found; using default: %s", mode_switch_topic_.c_str());
    }
    
    // Set the trigger point
    trigger_point_ << -320.0, 5.0, 17.0;
    
    // Get trigger distance threshold from parameter if available
    if (!nh_.getParam("/planner/trigger_distance_threshold", trigger_distance_threshold_)) {
        ROS_WARN("Param 'trigger_distance_threshold' not found; using default: %f", trigger_distance_threshold_);
    }

    // create publisher for RVIZ markers
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
    pub_trajectory_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);
    
    // Create publisher for mode switch with latched=true
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<std_msgs::Bool>(
        mode_switch_topic_, 1, // topic name, queue_size
        ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(), // connect_cb, disconnect_cb
        ros::VoidConstPtr(), NULL); // tracked_object, latched=true
    pub_mode_switch_ = nh.advertise(ao);
    fallback_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 1, true);
    // subscriber for Odometry
    sub_odom_ = nh.subscribe("/current_state_est", 1, &BasicPlanner::uavOdomCallback, this);
    
    // subscriber for goal positions
    sub_goal_position_ = nh.subscribe(goal_position_topic_, 1, &BasicPlanner::goalPositionCallback, this);
    
    ROS_INFO("Planner initialized. Will switch modes when within %f meters of point [%f, %f, %f]",
             trigger_distance_threshold_, trigger_point_[0], trigger_point_[1], trigger_point_[2]);
}

// Callback to get current Pose of UAV
void BasicPlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    // store current position in our planner
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);

    // store current velocity
    tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
    
    // Check if we're near the trigger point and should switch modes
    if (!mode_switched_ && isNearTriggerPoint()) {
        ROS_INFO("Reached trigger point! Switching to goal-based mode");
        mode_switched_ = true;
        goal_position_ = current_pose_.translation();
        // Publish mode switch message (latched)
        std_msgs::Bool mode_msg;
        mode_msg.data = true;
        pub_mode_switch_.publish(mode_msg);
        ROS_INFO("Published mode switch message to %s", mode_switch_topic_.c_str());
    }
}

// Check if current position is close to the trigger point
bool BasicPlanner::isNearTriggerPoint() {
    Eigen::Vector3d current_position = current_pose_.translation();
    double distance = (current_position - trigger_point_).norm();
    
    if (distance < trigger_distance_threshold_) {
        ROS_INFO("Current distance to trigger point: %f (threshold: %f)",
                 distance, trigger_distance_threshold_);
        return true;
    }
    return false;
}

// Method to set maximum speed.
void BasicPlanner::setMaxSpeed(const double max_v) {
    max_v_ = max_v;
}

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
bool BasicPlanner::planTrajectory(const Eigen::VectorXd& goal_pos,
                                     const Eigen::VectorXd& goal_vel,
                                     mav_trajectory_generation::Trajectory* trajectory) {

    // 3 Dimensional trajectory => through carteisan space, no orientation
    const int dimension = 3;

    // Array for all waypoints and their constrains
    mav_trajectory_generation::Vertex::Vector vertices;

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::ACCELERATION;

    // we have 2 vertices:
    // Start = current position
    // end = desired position and velocity
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);


    /******* Configure start point *******/
    // set start point constraints to current position and set all derivatives to zero
    start.makeStartOrEnd(current_pose_.translation(),
                         derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        current_velocity_);

    // add waypoint to list
    vertices.push_back(start);

    /******* Configure trajectory *******/
    // Only add waypoints from parameter space if mode has not been switched
    if (!mode_switched_) {
        ROS_INFO("Using waypoints from parameter space");

        // Publish mode switch message (latched)
        std_msgs::Bool mode_msg;
        mode_msg.data = false;
        pub_mode_switch_.publish(mode_msg);
        ROS_INFO("Published mode switch message to %s", mode_switch_topic_.c_str());

        XmlRpc::XmlRpcValue waypoint_list;
        if (nh_.getParam("/planner/waypoints/vertices", waypoint_list)) {
            // Parse the waypoints
            for (int i = 0; i < waypoint_list.size(); ++i) {
                mav_trajectory_generation::Vertex waypoint(dimension);

                // Add position constraint
                Eigen::Vector3d position;
                for (int j = 0; j < dimension; ++j) {
                    position[j] = static_cast<double>(waypoint_list[i]["position"][j]);
                }
                waypoint.addConstraint(mav_trajectory_generation::derivative_order::POSITION, position);

                // // Add velocity constraint if available
                // if (waypoint_list[i].hasMember("velocity")) {
                //     Eigen::Vector3d velocity;
                //     for (int j = 0; j < dimension; ++j) {
                //         velocity[j] = static_cast<double>(waypoint_list[i]["velocity"][j]);
                //     }
                //     waypoint.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, velocity);
                // }

                // // Add acceleration constraint if available
                // if (waypoint_list[i].hasMember("acceleration")) {
                //     Eigen::Vector3d acceleration;
                //     for (int j = 0; j < dimension; ++j) {
                //         acceleration[j] = static_cast<double>(waypoint_list[i]["acceleration"][j]);
                //     }
                //     waypoint.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, acceleration);
                // }

                // Add waypoint to vertices
                vertices.push_back(waypoint);
            }
        }
    } else {
        ROS_INFO("Creating direct trajectory to goal position");
        // When mode is switched, create a direct trajectory from current position to goal
        // No intermediate waypoints needed
    }

    /******* Configure end point *******/
    // set end point constraints to desired position and set all derivatives to zero
    end.makeStartOrEnd(goal_pos,
                       derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      goal_vel);

    // add waypoint to list
    vertices.push_back(end);

    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;

    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));

    return true;
}

bool BasicPlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance = 0.2; // Distance by which to separate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";

    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    pub_markers_.publish(markers);

    // send trajectory to be executed on UAV
    mav_planning_msgs::PolynomialTrajectory4D msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
    msg.header.frame_id = "world";
    pub_trajectory_.publish(msg);

    return true;
}


// Callback for new goal positions
void BasicPlanner::goalPositionCallback(const geometry_msgs::Point::ConstPtr& goal) {
    goal_position_[0] = goal->x;
    goal_position_[1] = goal->y;
    goal_position_[2] = goal->z;
    
    // Set goal velocity to zero
    goal_velocity_ = Eigen::Vector3d::Zero();
    
    ROS_INFO("New goal received: [%f, %f, %f]", goal_position_[0], goal_position_[1], goal_position_[2]);
    
    // Only set new_goal_received flag if mode has been switched
    // This ensures we only plan new trajectories in Mode 2
    if (mode_switched_) {
        new_goal_received_ = true;
    }
}
#include <cstdlib>  // for system()

void BasicPlanner::publishFallbackTrajectory() {
    // Kill the conflicting node
    int ret = system("rosnode kill /trajectory_converter");
    if(ret == 0) {
        ROS_INFO("Successfully killed /trajectory_converter node.");
    } else {
        ROS_WARN("Failed to kill /trajectory_converter node.");
    }

    // Now publish the fallback desired state
    trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
    tf::Vector3 origin(-320, 5, 17);
    tf::Vector3 displacement(0, 0, 0);  // For STATIC_POSE mode

    tf::Transform desired_pose(tf::Transform::getIdentity());
    desired_pose.setOrigin(origin + displacement);
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    desired_pose.setRotation(q);

    msg.transforms.resize(1);
    msg.transforms[0].translation.x = desired_pose.getOrigin().x();
    msg.transforms[0].translation.y = desired_pose.getOrigin().y();
    msg.transforms[0].translation.z = desired_pose.getOrigin().z();
    msg.transforms[0].rotation.x = desired_pose.getRotation().x();
    msg.transforms[0].rotation.y = desired_pose.getRotation().y();
    msg.transforms[0].rotation.z = desired_pose.getRotation().z();
    msg.transforms[0].rotation.w = desired_pose.getRotation().w();

    // Zero velocities/accelerations
    geometry_msgs::Twist zero_twist;
    zero_twist.linear.x = zero_twist.linear.y = zero_twist.linear.z = 0;
    zero_twist.angular.x = zero_twist.angular.y = zero_twist.angular.z = 0;
    msg.velocities.resize(1, zero_twist);
    msg.accelerations.resize(1, zero_twist);

    fallback_pub_.publish(msg);
    ROS_INFO("Published fallback desired state at static pose.");
}

// Main run method to handle both modes
void BasicPlanner::run() {
    ros::Rate rate(10); // 10 Hz
    bool param_trajectory_published = false;
    
    while (ros::ok()) {
        if (!mode_switched_ && !param_trajectory_published) {
            // Declare trajectory variable before using it
            mav_trajectory_generation::Trajectory trajectory;
            if (planTrajectory(goal_position_, goal_velocity_, &trajectory)) {
                publishTrajectory(trajectory);
                param_trajectory_published = true;
            }
        } 
        else if (mode_switched_) {
            if (new_goal_received_) {
                // Declare trajectory variable here as well
                mav_trajectory_generation::Trajectory trajectory;
                if (planTrajectory(goal_position_, goal_velocity_, &trajectory)) {
                    publishTrajectory(trajectory);
                    new_goal_received_ = false;
                }
            } 
            else {
                // Fallback: No new goal, so publish a default desired state
                publishFallbackTrajectory();
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

