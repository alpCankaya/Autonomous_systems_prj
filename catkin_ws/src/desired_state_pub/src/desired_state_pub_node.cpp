#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

void setNonBlockingMode(bool enable)
{
    static struct termios oldt;
    struct termios newt;

    if (enable)
    {
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }
    else
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) & ~O_NONBLOCK);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "desired_state_pub");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("/desired_state", 10, true);

    trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
    geometry_msgs::Transform transform;
    
    transform.rotation.x = 0.0;
    transform.rotation.y = 0.0;
    transform.rotation.z = 1.0;
    transform.rotation.w = 0.0;

    double x = -38.0, y = 10.0, z = 10.0;
    transform.translation.x = x;
    transform.translation.y = y;
    transform.translation.z = z;

    msg.transforms.push_back(transform);
    msg.velocities.resize(1);
    msg.accelerations.resize(1);
    
    pub.publish(msg);
    ROS_INFO("Initial desired state published: x=%.2f, y=%.2f, z=%.2f", x, y, z);

    setNonBlockingMode(true);
    ros::Rate loop_rate(10);
    double yaw = M_PI;

    while (ros::ok())
    {
        char c;
        int n = read(STDIN_FILENO, &c, 1);
        if (n > 0)
        {
            if (c == '\x1b')
            {
                char seq[2];
                if (read(STDIN_FILENO, &seq[0], 1) > 0 && read(STDIN_FILENO, &seq[1], 1) > 0)
                {
                    if (seq[0] == '[')
                    {
                        switch (seq[1])
                        {
                            case 'A': x -= 1.0; break;
                            case 'B': x += 1.0; break;
                            case 'C': y += 1.0; break;
                            case 'D': y -= 1.0; break;
                        }
                    }
                }
            }
            else if (c == 'q' || c == 'Q')
                break;
            else if (c == 'E' || c == 'e')
                yaw += 0.1;
            else if (c == 'F' || c == 'f')
                yaw -= 0.1;
            else if (c == 'W' || c == 'w')
                z += 1.0;
            else if (c == 'S' || c == 's')
                z -= 1.0;
        }

        transform.rotation.z = sin(yaw / 2.0);
        transform.rotation.w = cos(yaw / 2.0);

        msg.transforms[0].translation.x = x;
        msg.transforms[0].translation.y = y;
        msg.transforms[0].translation.z = z;
        msg.transforms[0].rotation.z = transform.rotation.z;
        msg.transforms[0].rotation.w = transform.rotation.w;

        pub.publish(msg);
        ROS_INFO("Published updated state: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", x, y, z, yaw);

        ros::spinOnce();
        loop_rate.sleep();
    }

    setNonBlockingMode(false);
    return 0;
}
