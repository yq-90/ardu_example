#include "control/control.hpp"

#include <memory>

/*
 * Drone states
 */

bool ControlNode::connected() const
{
    return this->current_state_.connected;
}

bool ControlNode::isGuided() const
{
    return this->current_state_.guided;
}

bool ControlNode::armed() const
{
    return this->current_state_.armed;
}

const std::string &ControlNode::getMode() const
{
    return this->current_state_.mode;
}

/*
 * Drone operations
 */

ControlNode::SetModeResultTy ControlNode::setMode(const std::string &mode) const
{
    auto request = std::make_shared<SetModeRequestTy>();
    request->custom_mode = mode.c_str();
    return this->setting_mode_client_->async_send_request(request).future.share();
}

ControlNode::SetModeResultTy ControlNode::setMode(uint8_t mode) const
{
    auto request = std::make_shared<SetModeRequestTy>();
    request->base_mode = mode;
    return this->setting_mode_client_->async_send_request(request).future.share();
}

ControlNode::ArmingResultTy ControlNode::arm() const
{
    auto request = std::make_shared<ArmingRequestTy>();
    request->value = true;
    return this->arming_client_->async_send_request(request).future.share();
}

ControlNode::TakeoffResultTy ControlNode::takeoff(
        const struct ControlNode::TakeoffConfig &config) const
{
    auto request = std::make_shared<TakeoffRequestTy>();
    request->min_pitch = config.min_pitch;
    request->yaw       = config.yaw;
    request->latitude  = config.latitude;
    request->longitude = config.longitude;
    request->altitude  = config.altitude;
    return this->takeoff_client_->async_send_request(request).future.share();
}

void ControlNode::setLocalTargetCoordinate(double x, double y, double z)
{
    auto tgt_coord_msg = geometry_msgs::msg::PoseStamped();
    tgt_coord_msg.pose.position.x = x;
    tgt_coord_msg.pose.position.y = y;
    tgt_coord_msg.pose.position.z = z;
    this->setpoint_local_publisher_->publish(tgt_coord_msg);
    return ;
}
