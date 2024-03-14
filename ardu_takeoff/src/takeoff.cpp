#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <chrono>

#include "takeoff/takeoff_control.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TakeoffControlNode>("takeoff_control");

    // Setting the retry interval to 200ms
    auto rate = rclcpp::Rate(200ms);

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node);

    // Test if autopilot is connected
    while (rclcpp::ok() && !node->connected()) {
        executor.spin_once();
        rate.sleep();
    }

    // Must sleep for a while before system is somehow boot up
    rclcpp::Rate(2s).sleep();

    // Step1: Set mode to GUIDED
    auto set_mode_res = node->setMode("GUIDED");
    if (executor.spin_until_future_complete(set_mode_res) == rclcpp::FutureReturnCode::SUCCESS &&
            set_mode_res.get()->mode_sent) {
        RCLCPP_INFO(rclcpp::get_logger("TakeoffNode"), "Successfully set mode to GUIDED");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("TakeoffNode"), "Failed to set mode to GUIDED");
        exit(EXIT_FAILURE);
    }

    /* 
     * Step2: Arm the drone
     *
     * As we have to wait until GPS has done calibration to arm, and there is no 
     * message to tell whether GPS has calibrated, we will have to retry arming until
     * the drone can be armed
     */

    // Wait for 35 seconds before trying to arm
    rclcpp::Rate(35s).sleep();

    do {
        auto arming_res = node->arm();
        executor.spin_until_future_complete(arming_res);
        rate.sleep();
    } while (rclcpp::ok() && !node->armed());

    // Prepare parameters to takeoff
    struct TakeoffControlNode::TakeoffConfig takeoff_config = {
        .min_pitch = 0,
        .yaw = 0,
        .latitude = 0,
        .longitude = 0,
        .altitude = 5};

    // Step3: Takeoff!
    auto takeoff_res = node->takeoff(takeoff_config);
    if (executor.spin_until_future_complete(takeoff_res) == rclcpp::FutureReturnCode::SUCCESS &&
            takeoff_res.get()->success) {
        RCLCPP_INFO(rclcpp::get_logger("TakeoffNode"), "Successfully takeoff");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("TakeoffNode"), "Failed to takeoff");
        exit(EXIT_FAILURE);
    }

    executor.spin();

    rclcpp::shutdown();

    return 0;
}
