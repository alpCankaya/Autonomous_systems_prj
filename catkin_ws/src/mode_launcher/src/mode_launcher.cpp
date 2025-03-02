#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cstdlib>  // system()

class ModeLauncher
{
public:
  ModeLauncher() : mode_active_(false)
  {

    mode_switch_sub_ = nh_.subscribe("mode_switch", 1, &ModeLauncher::modeSwitchCallback, this);
  }

  void modeSwitchCallback(const std_msgs::Bool::ConstPtr &msg)
  {
    if (msg->data && !mode_active_)
    {

      mode_active_ = true;
      ROS_INFO("mode_switch true: 'drone_frontier_exploration node started...");
      

      int ret = system("rosrun drone_frontier_exploration drone_frontier_exploration");
      ROS_INFO("Executable launch returned: %d", ret);
      

      ros::shutdown();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber mode_switch_sub_;
  bool mode_active_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mode_launcher");
  
  ModeLauncher launcher;
  
  ros::spin();
  return 0;
}
