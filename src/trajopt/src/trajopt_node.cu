#include <filesystem>
#include <chrono>
#include <fstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "indy7_msgs/msg/joint_state.hpp"
#include "indy7_msgs/msg/joint_trajectory.hpp"
#include "indy7_msgs/msg/joint_trajectory_point.hpp"
#include "trajopt_solver.cuh"
#include "csv_utils.h"

namespace msgs = indy7_msgs::msg;
using Clock = std::chrono::system_clock;
using TimePoint = std::chrono::time_point<Clock>;
using Duration = std::chrono::duration<double>;

class TrajoptNode : public rclcpp::Node
{
public:
    TrajoptNode(const std::string& traj_file)
    : Node("trajopt_node"), 
      timestep_(std::chrono::duration<double>(0.01)),
      pcg_exit_tol_(5e-4),
      pcg_max_iter_(173),
      state_updated_(false),
      warm_start_complete_(false),
      trajectory_start_time_()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing TrajoptNode");
        
        // Load trajectory from file
        auto goal_eePos_traj_2d = readCsvToVectorOfVectors<float>(traj_file);
        
        // Convert 2D trajectory to 1D
        std::vector<float> goal_eePos_traj_1d;
        for (const auto& vec : goal_eePos_traj_2d) {
            goal_eePos_traj_1d.insert(goal_eePos_traj_1d.end(), vec.begin(), vec.end());
        }

        // Initialize solver
        solver_ = std::make_unique<TrajoptSolver<float, 12, 6, 128, 128>>(
            goal_eePos_traj_1d,
            timestep_.count(),
            pcg_exit_tol_,
            pcg_max_iter_
        );

        // Create subscribers and publishers
        state_sub_ = create_subscription<msgs::JointState>(
            "joint_states", 1, 
            std::bind(&TrajoptNode::stateCallback, this, std::placeholders::_1)
        );

        traj_pub_ = create_publisher<msgs::JointTrajectory>("joint_trajectory", 1);

        // Wait for initial state
        RCLCPP_INFO(this->get_logger(), "Waiting for initial state...");
        while (rclcpp::ok() && !state_updated_) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        RCLCPP_INFO(this->get_logger(), "Initial state received");

        std::vector<float> pos_vec(current_state_.positions.begin(), current_state_.positions.end());
        solver_->initializeXU(pos_vec);
        RCLCPP_INFO(this->get_logger(), "Starting solver warm start...");
        solver_->warmStart();
        warm_start_complete_ = true;
        RCLCPP_INFO(this->get_logger(), "Solver warm start complete");

        // Pre-allocate message objects
        traj_msg_.knot_points = 128;
        traj_msg_.points.reserve(128);
        full_state_.reserve(12);  // 6 positions + 6 velocities
        
        // Pre-fill trajectory points to avoid allocations during runtime
        for (int i = 0; i < 128; i++) {
            msgs::JointTrajectoryPoint point;
            traj_msg_.points.emplace_back(point);
        }
    }

    ~TrajoptNode() {
        RCLCPP_INFO(this->get_logger(), "Shutting down TrajoptNode");
    }

private:
    void stateCallback(const msgs::JointState::SharedPtr msg)
    {
        // Skip if optimization is already running
        if (optimization_in_progress_.load()) {
            RCLCPP_DEBUG(this->get_logger(), "Skipping optimization - previous iteration still running");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_ = *msg;
            // Convert from degrees to radians
            for (int i = 0; i < 6; i++) {
                current_state_.positions[i] = current_state_.positions[i] * M_PI / 180.0;
                current_state_.velocities[i] = current_state_.velocities[i] * M_PI / 180.0;
            }
            state_updated_ = true;
        }

        if (!warm_start_complete_) {
            return;
        }

        // Early return if trajectory is complete
        if (solver_->isTrajectoryComplete()) {
            return;
        }

        optimization_in_progress_ = true;

        // Pre-allocate full_state with the correct size to avoid reallocations
        std::vector<float> full_state;
        full_state.reserve(current_state_.positions.size() * 2);  // Reserve space for positions + velocities
        full_state.insert(full_state.end(), 
            current_state_.positions.begin(), 
            current_state_.positions.end());
        full_state.insert(full_state.end(), 
            current_state_.velocities.begin(), 
            current_state_.velocities.end());

        // Convert ROS time to system_clock time
        auto now = Clock::now();
        if (trajectory_start_time_ == TimePoint()) {
            trajectory_start_time_ = now;
        }
        
        // Calculate elapsed time
        Duration elapsed = now - trajectory_start_time_;
        
        // Run optimization
        solver_->shiftTrajectory(full_state, elapsed.count());
        std::string stats = solver_->runTrajoptIteration();
        RCLCPP_INFO(this->get_logger(), "Optimization stats: %s", stats.c_str());
        
        RCLCPP_INFO(this->get_logger(), "Trajectory offset: %u", solver_->getTrajectoryOffset());

        publishTrajectory(msg->header.stamp);

        optimization_in_progress_ = false;
    }

    void publishTrajectory(const builtin_interfaces::msg::Time& stamp)
    {
        auto traj_msg = msgs::JointTrajectory();
        traj_msg.header.stamp = stamp;
        traj_msg.knot_points = 128;  // KNOT_POINTS
        traj_msg.dt = timestep_.count();  // Convert Duration to double

        // Get optimized trajectory from solver
        const auto [traj_data, traj_size] = solver_->getOptimizedTrajectory();
        
        // Populate trajectory points
        const int stride = 18;  // StateSize + ControlSize = 12 + 6
        const int num_joints = 6;  // StateSize/2
        
        traj_msg.points.reserve(traj_msg.knot_points);
        for (int i = 0; i < traj_msg.knot_points; i++) {
            msgs::JointTrajectoryPoint point;
            
            // Extract positions (first 6 values of state)
            for (size_t j = 0; j < 6; ++j) {
                point.positions[j] = traj_data[i * stride + j] * 180.0 / M_PI;
                point.velocities[j] = traj_data[i * stride + num_joints + j] * 180.0 / M_PI;
            }
            
            traj_msg.points.emplace_back(point);
        }

        traj_pub_->publish(traj_msg);
        RCLCPP_INFO(this->get_logger(), "Published trajectory: %f %f %f %f %f %f",
                    traj_msg.points[0].positions[0], traj_msg.points[0].positions[1], traj_msg.points[0].positions[2], traj_msg.points[0].positions[3], traj_msg.points[0].positions[4], traj_msg.points[0].positions[5]);
    }

    // Node members
    std::unique_ptr<TrajoptSolver<float, 12, 6, 128, 128>> solver_;
    rclcpp::Subscription<msgs::JointState>::SharedPtr state_sub_;
    rclcpp::Publisher<msgs::JointTrajectory>::SharedPtr traj_pub_;

    const Duration timestep_;
    const float pcg_exit_tol_;
    const int pcg_max_iter_;

    msgs::JointState current_state_;
    std::atomic<bool> state_updated_;
    std::mutex state_mutex_;
    TimePoint trajectory_start_time_;
    std::atomic<bool> warm_start_complete_;
    std::atomic<bool> optimization_in_progress_{false};

    // Reuse these objects instead of creating new ones each callback
    msgs::JointTrajectory traj_msg_;
    std::vector<float> full_state_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajoptNode>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}