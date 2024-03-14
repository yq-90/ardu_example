#ifndef TAKEOFF_CONTROL_HPP
#define TAKEOFF_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"

using namespace std::chrono_literals;

const std::string &get_state_topic();
const std::string &get_set_mode_service();
const std::string &get_arming_service();
const std::string &get_takeoff_service();

template <typename T>
void wait_client_init(T client)
{
    while (!(client->wait_for_service(1s))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("TakeoffNode"),
                    "Interrupted while waiting for the service. Exiting.");
            exit(EXIT_FAILURE);
        }
        RCLCPP_INFO(rclcpp::get_logger("TakeoffNode"),
                "service not available, waiting again...");
    }
}

class TakeoffControlNode : public rclcpp::Node {
    public:
        using SetModeInterface = mavros_msgs::srv::SetMode;
        using SetModeRequestTy = SetModeInterface::Request;
        using SetModeClientTy = rclcpp::Client<SetModeInterface>;
        using SetModeResultTy = SetModeClientTy::SharedFuture;

        using ArmingInterface = mavros_msgs::srv::CommandBool;
        using ArmingClientTy = rclcpp::Client<ArmingInterface>;
        using ArmingRequestTy = ArmingInterface::Request;
        using ArmingResultTy = ArmingClientTy::SharedFuture;

        using TakeoffInterface = mavros_msgs::srv::CommandTOL;
        using TakeoffClientTy = rclcpp::Client<TakeoffInterface>;
        using TakeoffRequestTy = TakeoffInterface::Request;
        using TakeoffResultTy = TakeoffClientTy::SharedFuture;

        // FIXME Maybe a fragile base class problem
        TakeoffControlNode(const std::string &name) : Node(name) {
            auto state_cb = [this] (const mavros_msgs::msg::State &msg) -> void {
                this->current_state_ = msg;
                return ;
            };
            state_subscription_ = this->create_subscription<mavros_msgs::msg::State>(
                    get_state_topic(), 10, state_cb);

            setting_mode_client_ =
                this->create_client<SetModeInterface>(get_set_mode_service());
            if (!setting_mode_client_) {
                RCLCPP_ERROR(rclcpp::get_logger("TakeoffNode"),
                        "Failed to instantialize set mode client");
                exit(EXIT_FAILURE);
            }
            wait_client_init(setting_mode_client_);

            arming_client_ =
                this->create_client<ArmingInterface>(get_arming_service());
            if (!arming_client_) {
                RCLCPP_ERROR(rclcpp::get_logger("TakeoffNode"),
                        "Failed to instantialize arm client");
                exit(EXIT_FAILURE);
            }
            wait_client_init(arming_client_);

            takeoff_client_ =
                this->create_client<TakeoffInterface>(get_takeoff_service());
            if (!takeoff_client_) {
                RCLCPP_ERROR(rclcpp::get_logger("TakeoffNode"),
                        "Failed to instantialize takeoff client");
                exit(EXIT_FAILURE);
            }
            wait_client_init(takeoff_client_);
        }

        bool connected() const;
        bool isGuided() const;
        bool armed() const;
        const std::string &getMode() const;

        SetModeResultTy setMode(const std::string &mode) const;
        SetModeResultTy setMode(uint8_t mode) const;

        ArmingResultTy arm() const;

        struct TakeoffConfig {
            float min_pitch;
            float yaw;
            float latitude;
            float longitude;
            float altitude;
        };
        TakeoffResultTy takeoff(const struct TakeoffConfig &config) const;
    private:
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
        mavros_msgs::msg::State current_state_;

        SetModeClientTy::SharedPtr setting_mode_client_;
        ArmingClientTy::SharedPtr arming_client_;
        TakeoffClientTy::SharedPtr takeoff_client_;
};

#endif
