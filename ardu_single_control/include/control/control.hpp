#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "control/ros_name.hpp"

using namespace std::chrono_literals;

template <typename T>
void wait_client_init(T client)
{
    while (!(client->wait_for_service(1s))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("ControlNode"),
                    "Interrupted while waiting for the service. Exiting.");
            exit(EXIT_FAILURE);
        }
        RCLCPP_INFO(rclcpp::get_logger("ControlNode"),
                "service not available, waiting again...");
    }
}

class ControlNode : public rclcpp::Node {
    public:
        // FIXME Oh FUCK ME. rclcpp::Node does not even has a type parameter for its shared_ptr
        // Worse even, it would fuck up the binary size if we adapt a template for ControlNode
        using SharedPtr = std::shared_ptr<ControlNode>;

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
        ControlNode(const std::string &name) : Node(name) {
            auto state_cb = [this] (const mavros_msgs::msg::State &msg) -> void {
                this->current_state_ = msg;
                return ;
            };
            state_subscription_ = this->create_subscription<mavros_msgs::msg::State>(
                    get_state_topic(), rclcpp::SystemDefaultsQoS(), state_cb);

            setting_mode_client_ =
                this->create_client<SetModeInterface>(get_set_mode_service());
            if (!setting_mode_client_) {
                RCLCPP_ERROR(this->get_logger(),
                        "Failed to instantialize set mode client");
                exit(EXIT_FAILURE);
            }

            arming_client_ =
                this->create_client<ArmingInterface>(get_arming_service());
            if (!arming_client_) {
                RCLCPP_ERROR(this->get_logger(),
                        "Failed to instantialize arm client");
                exit(EXIT_FAILURE);
            }
            takeoff_client_ =
                this->create_client<TakeoffInterface>(get_takeoff_service());
            if (!takeoff_client_) {
                RCLCPP_ERROR(this->get_logger(),
                        "Failed to instantialize takeoff client");
                exit(EXIT_FAILURE);
            }

            wait_client_init(setting_mode_client_);
            wait_client_init(arming_client_);
            wait_client_init(takeoff_client_);

            setpoint_local_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                    get_setpoint_local_topic(), rclcpp::SystemDefaultsQoS());

            auto update_pose = [this] (const geometry_msgs::msg::PoseStamped &msg) -> void {
                this->current_pose_ = msg.pose;
                return ;
            };
            local_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    get_local_pose_topic(), rclcpp::SensorDataQoS(), update_pose);

            auto move_to_rviz_goal = [this] (const geometry_msgs::msg::PoseStamped &msg) -> void {
                auto goal_pose_msg = geometry_msgs::msg::PoseStamped();
                goal_pose_msg.pose = msg.pose;
                goal_pose_msg.pose.position.z = this->current_pose_.position.z;
                this->setpoint_local_publisher_->publish(goal_pose_msg);
                return ;
            };
            rviz_goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    get_rviz_goal_topic(), rclcpp::SystemDefaultsQoS(), move_to_rviz_goal);
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

        void setLocalTargetCoordinate(double x, double y, double z);
    private:
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
        mavros_msgs::msg::State current_state_;

        SetModeClientTy::SharedPtr setting_mode_client_;
        ArmingClientTy::SharedPtr arming_client_;
        TakeoffClientTy::SharedPtr takeoff_client_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_local_publisher_;
        rclcpp::TimerBase::SharedPtr setpoint_local_timer_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_subscription_;
        geometry_msgs::msg::Pose current_pose_;

        // rviz_goal is a 2d_goal
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_goal_subscription_;
};

#endif
