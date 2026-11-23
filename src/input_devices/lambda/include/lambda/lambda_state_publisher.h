#pragma once

#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <future>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <testbed_msgs/msg/lambda_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace lambda {
class LamdaStatePublisher : public rclcpp::Node {
public:
    LamdaStatePublisher();
    bool start();
    void stop();
    ~LamdaStatePublisher();
private:
    rclcpp::Publisher<testbed_msgs::msg::LambdaState>::SharedPtr state_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<std::thread> control_thread_;
    std::atomic<bool> running{false};
    bool fast_init = false;
    std::string hand_;
    std::string device_frame_;
    int device_index_ = -1;
    int device_serial_ = -1;
    const double poseUpdateInterval_ = 0.01;
    double fx = 0.;
    double fy = 0.;
    double fz = 0.;
    double tx = 0.;
    double ty = 0.;
    double tz = 0.;
    double fg = 0.;     //TODO: add gripper force subscription
    std::mutex force_mutex_;
    std::chrono::steady_clock::time_point last_force_update_{};
    double force_timeout_seconds_ = 0.1;
    int openDevice();
    void control(std::promise<bool>& has_initialized);
    void setForce(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
};
}
