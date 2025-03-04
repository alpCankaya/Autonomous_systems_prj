#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include <cstdlib> // for system() function

// Define drone states
enum class DroneState {
  IDLE,
  PREDEFINED_PATH,
  AUTONOMOUS_FLIGHT,
  RETURN_TO_CAVE_ENTRANCE,
  RETURN_TO_BASE,
  LANDED
};

class DroneStateMachine {
public:
  DroneStateMachine() :
    nh_("~"),
    current_state_(DroneState::IDLE),
    reached_cave_entrance_(false),
    object_count_(0),
    current_velocity_(0.0),
    current_x_(0.0),
    current_y_(0.0),
    current_z_(0.0)
  {
    // Odom (pose + twist)
    sub_odom_ = nh_.subscribe("/current_state_est", 1, 
                              &DroneStateMachine::uavOdomCallback, this);

    // /mode_switch: std_msgs::Bool type
    mode_switch_sub_ = nh_.subscribe("/mode_switch", 1, 
                                     &DroneStateMachine::modeSwitchCallback, this);

    // Number of detected objects
    object_count_sub_ = nh_.subscribe("/detected_object_count", 1, 
                                      &DroneStateMachine::objectCountCallback, this);

    // Topic for publishing drone's current state
    state_pub_ = nh_.advertise<std_msgs::String>("/drone_state", 1);

    // Emergency stop service
    emergency_srv_ = nh_.advertiseService("emergency_stop", 
                                          &DroneStateMachine::emergencyStopCallback, this);

    // Periodic loop (state check)
    state_timer_ = nh_.createTimer(ros::Duration(0.1), 
                                   &DroneStateMachine::stateLoop, this);

    ROS_INFO("DroneStateMachine started. Current state: IDLE");
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_odom_;
  ros::Subscriber mode_switch_sub_;
  ros::Subscriber object_count_sub_;

  ros::Publisher  state_pub_;
  ros::Timer      state_timer_;
  ros::ServiceServer emergency_srv_;

  // Variables for the state machine
  DroneState current_state_;
  bool       reached_cave_entrance_;
  int        object_count_;
  double     current_velocity_;

  // Current position
  double current_x_;
  double current_y_;
  double current_z_;

  //--- CALLBACK FUNCTIONS ---

  // Odom message with pose and twist data
  void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;

    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    double vz = msg->twist.twist.linear.z;
    current_velocity_ = std::sqrt(vx*vx + vy*vy + vz*vz);
  }

  // /mode_switch callback: std_msgs::Bool
  void modeSwitchCallback(const std_msgs::Bool::ConstPtr &msg)
  {
    // If it wasn't active before and the incoming message is true
    if (msg->data && !reached_cave_entrance_)
    {
      reached_cave_entrance_ = true;  
      ROS_INFO("mode_switch true: 'drone_frontier_exploration' node started...");

      // When starting the node, ensure a fixed node name so rosnode kill can work properly
      int ret = system("rosrun drone_frontier_exploration drone_frontier_exploration __name:=drone_frontier_exploration");
      ROS_INFO("Executable launch returned: %d", ret);

      // Now this node does not shut itself down (ros::shutdown() removed).
    }
  }

  // If object_count_ == 4, go from AUTONOMOUS_FLIGHT -> RETURN_TO_CAVE_ENTRANCE
  void objectCountCallback(const std_msgs::Int32& msg)
  {
    object_count_ = msg.data;
  }

  // Emergency stop service
  bool emergencyStopCallback(std_srvs::Empty::Request &req, 
                             std_srvs::Empty::Response &res)
  {
    ROS_WARN("Emergency stop service called -> IDLE");
    changeState(DroneState::IDLE);
    return true;
  }

  //--- STATE LOOP ---
  void stateLoop(const ros::TimerEvent& event)
  {
    switch (current_state_) 
    {
      case DroneState::IDLE:
      {
        if (current_velocity_ > 0.01) {
          changeState(DroneState::PREDEFINED_PATH);
        }
        break;
      }

      case DroneState::PREDEFINED_PATH:
      {
        if (reached_cave_entrance_) {
          changeState(DroneState::AUTONOMOUS_FLIGHT);
        }
        break;
      }

      case DroneState::AUTONOMOUS_FLIGHT:
      {
        if (object_count_ == 4) {
          changeState(DroneState::RETURN_TO_CAVE_ENTRANCE);
        }
        break;
      }

      case DroneState::RETURN_TO_CAVE_ENTRANCE:
      {
        
        if (arrivedCaveEntrance()) 
        {
          // Here we terminate the drone_frontier_exploration node:
          int retKill = system("rosnode kill /drone_frontier_exploration");
          ROS_INFO("Killed drone_frontier_exploration node, returned code: %d", retKill);

          changeState(DroneState::RETURN_TO_BASE);
        }
        break;
      }

      case DroneState::RETURN_TO_BASE:
      {
        if (arrivedBase()) {
          changeState(DroneState::LANDED);
        }
        break;
      }

      case DroneState::LANDED:
      {
        // Mission completion, etc.
        break;
      }
    }

    // Let's publish the current state
    publishState();
  }

  //--- HELPER FUNCTIONS ---

  void changeState(DroneState new_state)
  {
    current_state_ = new_state;
    ROS_INFO("Drone state changed -> %s", stateToString(current_state_).c_str());
  }

  std::string stateToString(DroneState state)
  {
    switch (state) {
      case DroneState::IDLE:                    return "IDLE";
      case DroneState::PREDEFINED_PATH:         return "PREDEFINED_PATH";
      case DroneState::AUTONOMOUS_FLIGHT:       return "AUTONOMOUS_FLIGHT";
      case DroneState::RETURN_TO_CAVE_ENTRANCE: return "RETURN_TO_CAVE_ENTRANCE";
      case DroneState::RETURN_TO_BASE:          return "RETURN_TO_BASE";
      case DroneState::LANDED:                return "LANDED";
      default:                                  return "UNKNOWN";
    }
  }

  void publishState()
  {
    std_msgs::String msg;
    msg.data = stateToString(current_state_);
    state_pub_.publish(msg);
  }

  bool arrivedCaveEntrance()
  {
    const double cave_x = -320.0;
    const double cave_y = 5.0;
    const double cave_z = 17.0;
    const double threshold = 1.0;

    double dx = current_x_ - cave_x;
    double dy = current_y_ - cave_y;
    double dz = current_z_ - cave_z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    return (dist < threshold);
  }

  bool arrivedBase()
  {
    const double base_x = -38.0;
    const double base_y = 10.0;
    const double base_z = 6.0;
    const double threshold = 1.0;

    double dx = current_x_ - base_x;
    double dy = current_y_ - base_y;
    double dz = current_z_ - base_z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    return (dist < threshold);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_state_machine_node");
  DroneStateMachine machine;
  ros::spin();
  return 0;
}
