#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <chrono>

#include "control/control.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono_literals;

ControlNode::SharedPtr ardu_arm(rclcpp::executors::SingleThreadedExecutor &executor,
        ControlNode::SharedPtr ctrl_node,
        struct ControlNode::TakeoffConfig takeoff_config)
{

    executor.add_node(ctrl_node);

    // Setting the retry polling interval to 2s
    auto rate = rclcpp::Rate(2s);

    // Test if autopilot is connected
    while (rclcpp::ok() && !ctrl_node->connected()) {
        executor.spin_once();
        rate.sleep();
    }

    // Must sleep for a while before system is somehow boot up
    rate.sleep();

    // Step1: Set mode to GUIDED
    auto set_mode_res = ctrl_node->setMode("GUIDED");

    if (executor.spin_until_future_complete(set_mode_res) == rclcpp::FutureReturnCode::SUCCESS &&
            set_mode_res.get()->mode_sent) {
        RCLCPP_INFO(rclcpp::get_logger("ControlNode"), "Successfully set mode to GUIDED");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ControlNode"), "Failed to set mode to GUIDED");
        exit(EXIT_FAILURE);
    }

    // Step2: Arm the drone

    rclcpp::Rate(35s).sleep();

    do {
        auto arming_res = ctrl_node->arm();
        executor.spin_until_future_complete(arming_res);
        rate.sleep();
    } while (rclcpp::ok() && !ctrl_node->armed());

    // Step3: Takeoff!
    auto takeoff_res = ctrl_node->takeoff(takeoff_config);
    if (executor.spin_until_future_complete(takeoff_res) == rclcpp::FutureReturnCode::SUCCESS &&
            takeoff_res.get()->success) {
        RCLCPP_INFO(rclcpp::get_logger("TakeoffNode"), "Successfully takeoff");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("TakeoffNode"), "Failed to takeoff");
        exit(EXIT_FAILURE);
    }

    return ctrl_node;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    auto ctrl_node = std::make_shared<ControlNode>("control");

    struct ControlNode::TakeoffConfig takeoff_config = {.min_pitch = 0,
        .yaw = 0,
        .latitude = 0,
        .longitude = 0,
        .altitude = 5};
    ardu_arm(executor, ctrl_node, takeoff_config);

    rclcpp::Rate(20s).sleep();
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
