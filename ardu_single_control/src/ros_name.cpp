#include "control/ros_name.hpp"

/*
 * The following functions are to provide singletons of topic / service names
 */
const std::string &get_state_topic()
{
    static const std::string mavros_state_topic("/mavros/state");
    return mavros_state_topic;
}

const std::string &get_set_mode_service()
{
    static const std::string set_mode_service("/mavros/set_mode");
    return set_mode_service;
}

const std::string &get_arming_service()
{
    static const std::string arming_service("/mavros/cmd/arming");
    return arming_service;
}

const std::string &get_takeoff_service()
{
    static const std::string takeoff_service("/mavros/cmd/takeoff");
    return takeoff_service;
}

const std::string &get_setpoint_local_topic()
{
    static const std::string pose_topic("/mavros/setpoint_position/local");
    return pose_topic;
}

const std::string &get_setpoint_global_topic()
{
    static const std::string pose_topic("/mavros/setpoint_position/global");
    return pose_topic;
}

const std::string &get_local_pose_topic()
{
    static const std::string pose_topic("/mavros/local_position/pose");
    return pose_topic;
}

const std::string &get_rviz_goal_topic()
{
    static const std::string goal_topic("/goal_pose");
    return goal_topic;
}

