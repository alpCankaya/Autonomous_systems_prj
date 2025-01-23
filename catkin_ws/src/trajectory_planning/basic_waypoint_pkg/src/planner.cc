#include <planner.h>

BasicPlanner::BasicPlanner(ros::NodeHandle& nh) :
        nh_(nh),
        max_v_(0.2),
        max_a_(0.2),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity()) {

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

    // create publisher for RVIZ markers
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
    pub_trajectory_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);

    // subscriber for Odometry
    sub_odom_ = nh.subscribe("/current_state_est", 1, &BasicPlanner::uavOdomCallback, this);
}

// Callback to get current Pose of UAV
void BasicPlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

    // store current position in our planner
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);

    // store current velocity
    tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
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
            mav_trajectory_generation::derivative_order::SNAP;

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
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  To Do: Set up trajectory waypoints
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // In this section, you need to
    // - load waypoint definition (pos, vel, acc) per dimension from param file
    // - dynamically set constraints for each (and only where needed)
    // - push waypoints to vertices
    //
    // ~~~~ begin solution

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

            // Add velocity constraint if available
            if (waypoint_list[i].hasMember("velocity")) {
                Eigen::Vector3d velocity;
                for (int j = 0; j < dimension; ++j) {
                    velocity[j] = static_cast<double>(waypoint_list[i]["velocity"][j]);
                }
                waypoint.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, velocity);
            }

            // Add acceleration constraint if available
            if (waypoint_list[i].hasMember("acceleration")) {
                Eigen::Vector3d acceleration;
                for (int j = 0; j < dimension; ++j) {
                    acceleration[j] = static_cast<double>(waypoint_list[i]["acceleration"][j]);
                }
                waypoint.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, acceleration);
            }

            // Add waypoint to vertices
            vertices.push_back(waypoint);
        }
    }

    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //                                 end
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
