#include <chrono>
#include <functional>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>
#include <drdc.h>
#include <Eigen/Geometry>

#include "lambda/lambda_state_publisher.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace {
// Serialize discovery only. The SDK holds each selected device exclusively afterward.
class DiscoveryLock {
public:
    explicit DiscoveryLock(const std::atomic<bool>& running) {
        const std::string path = "/tmp/testbed-lambda-discovery-" + std::to_string(getuid()) + ".lock";
        fd_ = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (fd_ < 0) throw std::runtime_error("Cannot open Lambda discovery lock: " + std::string(strerror(errno)));
        struct stat metadata {};
        if (fstat(fd_, &metadata) != 0 || metadata.st_uid != getuid() || !S_ISREG(metadata.st_mode)) {
            close(fd_);
            fd_ = -1;
            throw std::runtime_error("Lambda discovery lock must be a regular file owned by this user");
        }
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
            if ((errno != EWOULDBLOCK && errno != EINTR) || !running || !rclcpp::ok() ||
                std::chrono::steady_clock::now() >= deadline) {
                close(fd_);
                fd_ = -1;
                throw std::runtime_error("Lambda discovery lock unavailable or startup cancelled");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    ~DiscoveryLock() {
        if (fd_ >= 0) { flock(fd_, LOCK_UN); close(fd_); }
    }
    DiscoveryLock(const DiscoveryLock&) = delete;
    DiscoveryLock& operator=(const DiscoveryLock&) = delete;
private:
    int fd_ = -1;
};

struct DeviceGuard {
    int id = -1;
    bool actuated = false;
    ~DeviceGuard() {
        if (id < 0) return;
        const auto handle = static_cast<char>(id);
        if (actuated) {
            dhdSetForceAndTorqueAndGripperForce(0., 0., 0., 0., 0., 0., 0., handle);
            drdStop(false, handle);
        }
        drdClose(handle);
    }
    int release() { const int result = id; id = -1; return result; }
    DeviceGuard() = default;
    explicit DeviceGuard(int handle) : id(handle) {}
    DeviceGuard(const DeviceGuard&) = delete;
    DeviceGuard& operator=(const DeviceGuard&) = delete;
};
}  // namespace

namespace lambda {
    LamdaStatePublisher::LamdaStatePublisher() :
        rclcpp::Node("lambda_state_publisher", "robot1")
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("lambda_state_topic", "lambda/state");
        this->declare_parameter<std::string>("lambda_force_topic", "lambda/force");
        this->declare_parameter<bool>("fast_init", false);
        force_timeout_seconds_ = declare_parameter<double>("force_timeout", 0.1);
        if (!std::isfinite(force_timeout_seconds_) || force_timeout_seconds_ <= 0.0) {
            throw std::invalid_argument("force_timeout must be finite and positive");
        }

        hand_ = declare_parameter<std::string>("hand", "any");
        device_index_ = declare_parameter<int>("device_index", -1);
        device_serial_ = declare_parameter<int>("device_serial", -1);
        if (hand_ != "any" && hand_ != "left" && hand_ != "right")
            throw std::invalid_argument("hand must be any, left or right");
        if (device_index_ < -1 || device_index_ > std::numeric_limits<char>::max())
            throw std::invalid_argument("device_index must be -1 or a valid SDK enumeration index");
        if (device_serial_ < -1 || device_serial_ > 65535)
            throw std::invalid_argument("device_serial must be -1 or a serial number from 0 to 65535");
        std::string prefix = get_namespace();
        if (!prefix.empty() && prefix.front() == '/') prefix.erase(0, 1);
        device_frame_ = declare_parameter<std::string>("device_frame", prefix.empty() ? "lambda_base" : prefix + "/lambda_base");
        if (device_frame_.empty() || device_frame_.front() == '/')
            throw std::invalid_argument("device_frame must be nonempty without a leading slash");

        // Get parameter values
        std::string state_topic = this->get_parameter("lambda_state_topic").as_string();
        std::string force_topic = this->get_parameter("lambda_force_topic").as_string();
        this->fast_init = this->get_parameter("fast_init").as_bool();

        // Create publisher and subscription with configurable topics
        state_publisher_ = create_publisher<testbed_msgs::msg::LambdaState>(state_topic, 10);
        force_subscription_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
            force_topic, 10, std::bind(&LamdaStatePublisher::setForce, this, _1));
        }

    LamdaStatePublisher::~LamdaStatePublisher() {
        stop();
    }

    int LamdaStatePublisher::openDevice() {
        DiscoveryLock discovery(running);
        const int count = dhdGetDeviceCount();
        if (count <= 0 || count > std::numeric_limits<char>::max() + 1)
            throw std::runtime_error("No Lambda devices found, or SDK enumeration failed");
        if (device_index_ >= count)
            throw std::runtime_error("Requested Lambda device_index is not connected");
        DeviceGuard selected;
        int selected_index = -1;
        for (int index = 0; index < count; ++index) {
            if (!running || !rclcpp::ok()) throw std::runtime_error("Lambda selection cancelled");
            if (device_index_ >= 0 && index != device_index_) continue;
            DeviceGuard candidate(drdOpenID(static_cast<char>(index)));
            if (candidate.id < 0) continue;  // SDK refuses devices owned by another application.
            const auto handle = static_cast<char>(candidate.id);
            const int type = dhdGetSystemType(handle);
            if (type != DHD_DEVICE_LAMBDA331 && type != DHD_DEVICE_LAMBDA331_LEFT) continue;
            const bool left = dhdIsLeftHanded(handle);
            if ((hand_ == "left" && !left) || (hand_ == "right" && left)) continue;
            unsigned short serial = 0;
            const bool has_serial = dhdGetSerialNumber(&serial, handle) >= DHD_NO_ERROR;
            if (device_serial_ >= 0 && (!has_serial || serial != device_serial_)) continue;
            if (selected.id >= 0)
                throw std::runtime_error("Multiple Lambda devices match; specify device_index or device_serial");
            selected.id = candidate.release();
            selected_index = index;
        }
        if (selected.id < 0)
            throw std::runtime_error("No available Lambda matches hand/device_index/device_serial; it may be busy");
        unsigned short serial = 0;
        const auto handle = static_cast<char>(selected.id);
        const bool has_serial = dhdGetSerialNumber(&serial, handle) >= DHD_NO_ERROR;
        RCLCPP_INFO(get_logger(), "Selected %s Lambda: index=%d handle=%d serial=%s frame=%s",
                    dhdIsLeftHanded(handle) ? "left" : "right", selected_index, selected.id,
                    has_serial ? std::to_string(serial).c_str() : "unavailable", device_frame_.c_str());
        return selected.release();
    }

    void LamdaStatePublisher::control(std::promise<bool>& has_initialized) {
        bool reported = false;
        try {
            DeviceGuard device_guard(openDevice());
            const auto device_id = static_cast<char>(device_guard.id);
            RCLCPP_INFO(get_logger(), "Opened lambda device: %s", dhdGetSystemName(device_id));
            if (!drdIsSupported(device_id)) {
                RCLCPP_ERROR(get_logger(), "Unsupported device type %s", dhdGetSystemName(device_id));
                has_initialized.set_value(false);
                return;
            }
            device_guard.actuated = true;
            if (!drdIsInitialized(device_id)) {
                RCLCPP_INFO(get_logger(), "Starting AutoInit.");
                if (drdAutoInit(device_id) < 0) {
                    RCLCPP_ERROR(get_logger(), "Failed to initialize device (%s)", dhdErrorGetLastStr());
                    has_initialized.set_value(false);
                    return;
                  }
            } else {
                RCLCPP_INFO(get_logger(), "Lambda device is already initialized. Skipping AutoInit.");
            }
            if (!this->fast_init) {
                RCLCPP_INFO(get_logger(), "Validating initialization");
                if (drdCheckInit(device_id) < 0) {
                    RCLCPP_ERROR(get_logger(), "Failed to validate device initialization (%s)", dhdErrorGetLastStr());
                    has_initialized.set_value(false);
                    return;
                }
            } else {
                RCLCPP_INFO(get_logger(), "Skipping validation of initialization since parameter fast_init=true");
                if (drdStart(device_id) < 0) {
                    RCLCPP_ERROR(get_logger(), "Failed to start robotic regulation (%s)", dhdErrorGetLastStr());
                    has_initialized.set_value(false);
                    return;
                }
                double positionCenter[DHD_MAX_DOF] = {0};
                if (drdMoveTo(positionCenter, true, device_id) < 0) {
                    RCLCPP_ERROR(get_logger(), "Failed to move the device to the center (%s)", dhdErrorGetLastStr());
                    has_initialized.set_value(false);
                    return;
                }
            }
            if (drdStop(true, device_id) < 0) {
                RCLCPP_ERROR(get_logger(), "Failed to stop robotic regulation (%s)", dhdErrorGetLastStr());
                has_initialized.set_value(false);
                return;
            }
            if (dhdEnableExpertMode() < DHD_NO_ERROR) {
                RCLCPP_ERROR(get_logger(),"Failed to enable expert mode (%s)", dhdErrorGetLastStr());
                has_initialized.set_value(false);
                return;
            }

            RCLCPP_INFO(get_logger(), "Initialization successful.");
            if (dhdSetEffectorMass(1.49021, device_id) < DHD_NO_ERROR)
                throw std::runtime_error("Failed to set Lambda effector mass");
            has_initialized.set_value(true);
            reported = true;
            // main control loop
            double px, py, pz, vx, vy, vz, wx, wy, wz, gripper_angle;

            double ee_fx = 0.0, ee_fy = 0.0, ee_fz = 0.0;
            double ee_tx = 0.0, ee_ty = 0.0, ee_tz = 0.0;
            double ee_fg = 0.0;

            double rotation[3][3];
            double joint_angles[DHD_MAX_DOF];
            double joint_velocities[DHD_MAX_DOF];

            double lastPoseUpdate = dhdGetTime();
            rclcpp::Rate rate(1000); // 1000 Hz
            while (running && rclcpp::ok()) {
                {
                    std::lock_guard<std::mutex> guard(force_mutex_);
                    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_force_update_).count() > force_timeout_seconds_) {
                        fx = fy = fz = tx = ty = tz = fg = 0.0;
                    }
                    if (dhdSetForceAndTorqueAndGripperForce(fx, fy, fz, tx, ty, tz, fg, device_id) < DHD_NO_ERROR) {
                        RCLCPP_ERROR(get_logger(), "Failed to set forces and torques (%s)", dhdErrorGetLastStr());
                    }
                }
                double time = dhdGetTime();
                if (time - lastPoseUpdate > poseUpdateInterval_) {
                    bool no_error = true;
                    if (dhdGetPosition(&px, &py, &pz, device_id) < DHD_NO_ERROR) {
                        RCLCPP_ERROR(get_logger(), "Failed to get position (%s)", dhdErrorGetLastStr());
                        no_error = false;
                    }
                    if (no_error && dhdGetOrientationFrame(rotation, device_id) < DHD_NO_ERROR) {
                        RCLCPP_ERROR(get_logger(), "Failed to get orientation (%s)", dhdErrorGetLastStr());
                        no_error = false;
                    }
                    if (no_error && dhdGetLinearVelocity(&vx, &vy, &vz, device_id) < DHD_NO_ERROR) {
                        RCLCPP_ERROR(get_logger(), "Failed to get linear velocity (%s)", dhdErrorGetLastStr());
                        no_error = false;
                    }
                    if (no_error && dhdGetAngularVelocityRad(&wx, &wy, &wz, device_id) < DHD_NO_ERROR) {
                        RCLCPP_ERROR(get_logger(), "Failed to get angular velocity (%s)", dhdErrorGetLastStr());
                        no_error = false;
                    }
                    if (no_error && dhdGetGripperAngleRad(&gripper_angle, device_id) < DHD_NO_ERROR) {
                        RCLCPP_ERROR(get_logger(), "Failed to get gripper angle (%s)", dhdErrorGetLastStr());
                        no_error = false;
                    }
                    if (no_error && dhdGetJointAngles(joint_angles, device_id) < DHD_NO_ERROR) {
                        RCLCPP_ERROR(get_logger(), "Failed to get wrist joint angles (%s)", dhdErrorGetLastStr());
                        no_error = false;
                    }
                    if (no_error && dhdGetJointVelocities(joint_velocities, device_id) < DHD_NO_ERROR) {
                        RCLCPP_ERROR(get_logger(), "Failed to get wrist joint velocities (%s)", dhdErrorGetLastStr());
                        no_error = false;
                    }
                    if (no_error && dhdGetForceAndTorqueAndGripperForce(&ee_fx, &ee_fy, &ee_fz, &ee_tx, &ee_ty, &ee_tz, &ee_fg, device_id) < DHD_NO_ERROR) {
                        RCLCPP_ERROR(get_logger(), "Failed to get wrist joint torques (%s)", dhdErrorGetLastStr());
                        no_error = false;
                    }

                    if (no_error){
                        testbed_msgs::msg::LambdaState msg;
                        msg.header.stamp = get_clock()->now();
                        msg.header.frame_id = device_frame_;
                        msg.pose.position.x = px;
                        msg.pose.position.y = py;
                        msg.pose.position.z = pz;
                        Eigen::Matrix3d rotation_eigen;
                        rotation_eigen << rotation[0][0], rotation[0][1], rotation[0][2],
                                          rotation[1][0], rotation[1][1], rotation[1][2],
                                          rotation[2][0], rotation[2][1], rotation[2][2];
                        Eigen::Quaterniond q(rotation_eigen);
                        msg.pose.orientation.x = q.x();
                        msg.pose.orientation.y = q.y();
                        msg.pose.orientation.z = q.z();
                        msg.pose.orientation.w = q.w();
                        msg.twist.linear.x = vx;
                        msg.twist.linear.y = vy;
                        msg.twist.linear.z = vz;
                        msg.twist.angular.x = wx;
                        msg.twist.angular.y = wy;
                        msg.twist.angular.z = wz;
                        msg.gripper_angle = gripper_angle;
                        std::lock_guard<std::mutex> force_guard(force_mutex_);
                        msg.commanded_end_effector_force.force.x = fx;
                        msg.commanded_end_effector_force.force.y = fy;
                        msg.commanded_end_effector_force.force.z = fz;
                        msg.commanded_end_effector_force.torque.x = tx;
                        msg.commanded_end_effector_force.torque.y = ty;
                        msg.commanded_end_effector_force.torque.z = tz;
                        msg.commanded_gripper_force = fg;
                        msg.joints.header = msg.header;

                        for (int i = 0; i < DHD_MAX_DOF; i++) {
                            msg.joints.name.push_back("joint" + std::to_string(i));
                            msg.joints.position.push_back(joint_angles[i]);
                            msg.joints.velocity.push_back(joint_velocities[i]);
                        }

                        msg.current_end_effector_force.force.x = ee_fx;
                        msg.current_end_effector_force.force.y = ee_fy;
                        msg.current_end_effector_force.force.z = ee_fz;
                        msg.current_end_effector_force.torque.x = ee_tx;
                        msg.current_end_effector_force.torque.y = ee_ty;
                        msg.current_end_effector_force.torque.z = ee_tz;

                        msg.current_gripper_force = ee_fg;

                        state_publisher_->publish(msg);
                    }
                    lastPoseUpdate = time;
                }
                try {
                    rate.sleep();
                } catch (const std::runtime_error&) {
                    // Shutdown can invalidate the context between the loop check and sleep.
                    if (!rclcpp::ok(get_node_base_interface()->get_context())) break;
                    throw;
                }
            }
        } catch (const std::exception& error) {
            RCLCPP_ERROR(get_logger(), "Lambda device failure: %s", error.what());
            if (!reported) has_initialized.set_value(false);
            else rclcpp::shutdown();
        }
        running = false;
    }

    bool LamdaStatePublisher::start() {
        running = true;
        std::promise<bool> p;
        std::future<bool> has_initialized = p.get_future();
        control_thread_ = std::make_shared<std::thread>(&LamdaStatePublisher::control, this, std::ref(p));
        has_initialized.wait();
        return has_initialized.get();
    }

    void LamdaStatePublisher::stop() {
        running = false;
        if (control_thread_ && control_thread_->joinable()) {
            control_thread_->join();
        }
    }

    void LamdaStatePublisher::setForce(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> guard(force_mutex_);
        if (!std::isfinite(msg->wrench.force.x) || !std::isfinite(msg->wrench.force.y) ||
            !std::isfinite(msg->wrench.force.z) || !std::isfinite(msg->wrench.torque.x) ||
            !std::isfinite(msg->wrench.torque.y) || !std::isfinite(msg->wrench.torque.z)) {
            return;
        }
        last_force_update_ = std::chrono::steady_clock::now();
        fx = msg->wrench.force.x;
        fy = msg->wrench.force.y;
        fz = msg->wrench.force.z;
        tx = msg->wrench.torque.x;
        ty = msg->wrench.torque.y;
        tz = msg->wrench.torque.z;
    }
}
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        auto node = std::make_shared<lambda::LamdaStatePublisher>();
        if (node->start()) {
            rclcpp::spin(node);
            node->stop();
        } else {
            RCLCPP_ERROR(node->get_logger(), "Could not start lambda node.");
            result = 1;
        }
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("lambda_state_publisher"), "%s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
