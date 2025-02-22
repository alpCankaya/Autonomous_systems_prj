#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// Function to set terminal to non-canonical, non-echo mode
void setNonBlockingMode(bool enable)
{
    static struct termios oldt;
    struct termios newt;

    if (enable)
    {
        // Get current terminal settings
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;

        // Disable canonical mode and echo
        newt.c_lflag &= ~(ICANON | ECHO);

        // Set new terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        // Set non-blocking read
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }
    else
    {
        // Restore old terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        // Set blocking read
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) & ~O_NONBLOCK);
    }
}

int main(int argc, char** argv)
{
    // Initialize ROS node with name "desired_state_pub"
    ros::init(argc, argv, "desired_state_pub");
    ros::NodeHandle nh;

    // Create a publisher for the "/desired_state" topic
    ros::Publisher pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
        "/desired_state", 10, true); // Increased queue size

    // Initialize the message
    trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
    geometry_msgs::Transform transform;

    // Set initial orientation to identity quaternion (no rotation)
    transform.rotation.x = 0.0;
    transform.rotation.y = 0.0;
    transform.rotation.z = -1.0;
    transform.rotation.w = 0.0;

    // Set initial translation to (-38, 10, 10)
    double x = -38.0;
    double y = 10.0;
    double z = 10.0;

    transform.translation.x = x;
    transform.translation.y = y;
    transform.translation.z = z;

    msg.transforms.push_back(transform);
    // Initialize empty velocities and accelerations
    msg.velocities.resize(1);      // One velocity for the transform
    msg.accelerations.resize(1);   // One acceleration for the transform

    // Publish the initial message
    pub.publish(msg);
    ROS_INFO("Initial desired state published: x=%.2f, y=%.2f, z=%.2f", x, y, z);

    // Set terminal to non-blocking mode
    setNonBlockingMode(true);
    ROS_INFO("Control started. Use Up Arrow to increment X, Left Arrow to increment Y, 'q' to quit.");

    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok())
    {
        char c;
        int n = read(STDIN_FILENO, &c, 1);
        if (n > 0)
        {
            if (c == '\x1b') // Escape character
            {
                char seq[2];
                if (read(STDIN_FILENO, &seq[0], 1) > 0 && read(STDIN_FILENO, &seq[1], 1) > 0)
                {
                    if (seq[0] == '[')
                    {
                        switch (seq[1])
                        {
                            case 'A': // Up Arrow
                                x -= 1.0;
                                ROS_INFO("Up Arrow pressed: Incremented X to %.2f", x);
                                break;
                            case 'B': // Down Arrow
                                x += 1.0;
                                ROS_INFO("Down Arrow pressed: Decremented X to %.2f", x);
                                break;
                            case 'C': // Right Arrow
                                y += 1.0;
                                ROS_INFO("Right Arrow pressed: Decremented Y to %.2f", y);
                                break;
                            case 'D': // Left Arrow
                                y -= 1.0;
                                ROS_INFO("Left Arrow pressed: Incremented Y to %.2f", y);
                                break;
                            default:
                                break;
                        }

                        // Update the message with new x and y
                        msg.transforms[0].translation.x = x;
                        msg.transforms[0].translation.y = y;
                        msg.transforms[0].translation.z = z;

                        // Publish the updated message
                        pub.publish(msg);
                        ROS_INFO("Published updated desired state: x=%.2f, y=%.2f, z=%.2f", x, y, z);
                    }
                }
            }
            else if (c == 'q' || c == 'Q')
            {
                ROS_INFO("Quit key pressed. Shutting down node.");
                break;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Restore terminal settings before exiting
    setNonBlockingMode(false);
    return 0;
}
