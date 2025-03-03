#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

class DesiredStateMux
{
public:
    DesiredStateMux()
    {
        ros::NodeHandle nh;

        // Subscribe to the two desired_state topics:
        // One from the trajectory_converter node...
        tc_sub_ = nh.subscribe("/tc_desired_state", 10, &DesiredStateMux::tcCallback, this);
        // ...and one from your fallback publisher.
        fallback_sub_ = nh.subscribe("/fallback_desired_state", 10, &DesiredStateMux::fallbackCallback, this);

        // Subscribe to a control topic that tells us which source to use.
        // true  -> use fallback (i.e. mode_switched true and no new goal)
        // false -> use trajectory_converter
        control_sub_ = nh.subscribe("/planner/mux_control", 10, &DesiredStateMux::controlCallback, this);

        // Publish the muxed desired_state for your controller.
        pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 10);

        // Create a timer to regularly publish the selected desired_state message.
        timer_ = nh.createTimer(ros::Duration(0.1), &DesiredStateMux::timerCallback, this);
    }

private:
    // Callback for messages from trajectory_converter.
    void tcCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg)
    {
        last_tc_ = *msg;
        tc_received_ = true;
    }

    // Callback for messages from the fallback publisher.
    void fallbackCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg)
    {
        last_fallback_ = *msg;
        fallback_received_ = true;
    }

    // Callback for the control flag.
    void controlCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        // If true, we want to use the fallback message.
        control_flag_ = msg->data;
    }

    // Timer callback to publish the selected desired state.
    void timerCallback(const ros::TimerEvent&)
    {
        // Choose which message to output based on the control flag.
        trajectory_msgs::MultiDOFJointTrajectoryPoint out_msg;
        if (control_flag_)  // Use fallback when control flag is true.
        {
            if (fallback_received_)
            {
                out_msg = last_fallback_;
            }
            else
            {
                ROS_WARN_THROTTLE(5, "No fallback desired state received yet.");
                return;
            }
        }
        else  // Otherwise, use the trajectory_converter output.
        {
            if (tc_received_)
            {
                out_msg = last_tc_;
            }
            else
            {
                ROS_WARN_THROTTLE(5, "No trajectory_converter desired state received yet.");
                return;
            }
        }
        pub_.publish(out_msg);
    }

    ros::Subscriber tc_sub_;
    ros::Subscriber fallback_sub_;
    ros::Subscriber control_sub_;
    ros::Publisher pub_;
    ros::Timer timer_;

    trajectory_msgs::MultiDOFJointTrajectoryPoint last_tc_;
    trajectory_msgs::MultiDOFJointTrajectoryPoint last_fallback_;
    bool tc_received_ = false;
    bool fallback_received_ = false;
    bool control_flag_ = false;  // false = use trajectory_converter, true = use fallback.
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "desired_state_mux");
    DesiredStateMux mux;
    ros::spin();
    return 0;
}
