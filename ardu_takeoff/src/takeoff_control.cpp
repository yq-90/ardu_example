#include "takeoff/takeoff_control.hpp"

#include <memory>

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

/*
 * Drone states
 */

bool TakeoffControlNode::connected() const
{
    return this->current_state_.connected;
}

bool TakeoffControlNode::isGuided() const
{
    return this->current_state_.guided;
}

bool TakeoffControlNode::armed() const
{
    return this->current_state_.armed;
}

const std::string &TakeoffControlNode::getMode() const
{
    return this->current_state_.mode;
}

/*
 * Drone operations
 */

TakeoffControlNode::SetModeResultTy TakeoffControlNode::setMode(const std::string &mode) const
{
    auto request = std::make_shared<SetModeRequestTy>();
    request->custom_mode = mode.c_str();
    return this->setting_mode_client_->async_send_request(request).future.share();
}

TakeoffControlNode::SetModeResultTy TakeoffControlNode::setMode(uint8_t mode) const
{
    auto request = std::make_shared<SetModeRequestTy>();
    request->base_mode = mode;
    return this->setting_mode_client_->async_send_request(request).future.share();
}

TakeoffControlNode::ArmingResultTy TakeoffControlNode::arm() const
{
    auto request = std::make_shared<ArmingRequestTy>();
    request->value = true;
    return this->arming_client_->async_send_request(request).future.share();
}

TakeoffControlNode::TakeoffResultTy TakeoffControlNode::takeoff(
        const struct TakeoffControlNode::TakeoffConfig &config) const
{
    auto request = std::make_shared<TakeoffRequestTy>();
    request->min_pitch = config.min_pitch;
    request->yaw       = config.yaw;
    request->latitude  = config.latitude;
    request->longitude = config.longitude;
    request->altitude  = config.altitude;
    return this->takeoff_client_->async_send_request(request).future.share();
}
