#ifndef ROS_NAME_HPP
#define ROS_NAME_HPP
#include <string>

const std::string &get_state_topic();
const std::string &get_set_mode_service();
const std::string &get_arming_service();
const std::string &get_takeoff_service();
const std::string &get_setpoint_local_topic();

const std::string &get_local_pose_topic();
const std::string &get_rviz_goal_topic();

#endif
