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
using SimTime = rclcpp::Time;

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
      trajectory_start_time_(),
      use_sim_time_(false)
    {
        use_sim_time_ = get_parameter("use_sim_time").as_bool();
        RCLCPP_INFO(this->get_logger(), "Using %s time", use_sim_time_ ? "simulation" : "system");

        RCLCPP_INFO(this->get_logger(), "Initializing TrajoptNode");
        state_sub_ = create_subscription<msgs::JointState>(
            "joint_states", 1, 
            std::bind(&TrajoptNode::stateCallback, this, std::placeholders::_1)
        );

        traj_pub_ = create_publisher<msgs::JointTrajectory>("joint_trajectory", 1);
        
        // Initialize solver with trajectory from file
        std::vector<float> goal_eePos_traj_1d = readCsvToVector<float>(traj_file);
        solver_ = std::make_unique<TrajoptSolver<float>>(
            goal_eePos_traj_1d,
            timestep_.count(),
            pcg_exit_tol_,
            pcg_max_iter_
        );
        RCLCPP_INFO(this->get_logger(), "Solver initialized");

        // Pre-allocate
        traj_msg_.knot_points = solver_->numKnotPoints();
        for (int i = 0; i < solver_->numKnotPoints(); i++) {
            msgs::JointTrajectoryPoint point;
            traj_msg_.points.emplace_back(point);
        }
        full_state_.reserve(solver_->stateSize());

        
        RCLCPP_INFO(this->get_logger(), "Waiting for initial state...");
        while (rclcpp::ok() && !state_updated_) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::vector<float> current_joint_positions(current_state_.positions.begin(), current_state_.positions.end());
        solver_->initializeXU(current_joint_positions);
        RCLCPP_INFO(this->get_logger(), "Received initial state, starting solver warm start...");
        solver_->warmStart();
        cudaDeviceSynchronize(); //cuda operations are asynchronous, so we need to synchronize here
        warm_start_complete_ = true;
        if (use_sim_time_) {
            // Wait for valid sim time
            while (rclcpp::ok() && current_state_.header.stamp.sec == 0) {
                rclcpp::spin_some(this->get_node_base_interface());
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            trajectory_start_stamp_ = current_state_.header.stamp;
        } else {
            trajectory_start_time_ = Clock::now();
        }
        RCLCPP_INFO(this->get_logger(), "Warm start complete");
    }

    ~TrajoptNode() {
        RCLCPP_INFO(this->get_logger(), "Shutting down TrajoptNode");
    }

private:
    void stateCallback(const std::shared_ptr<const msgs::JointState>& msg)
    {
        // Skip if optimization is already running or warm start is not complete
        if (optimization_in_progress_.load()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state_ = *msg;
            // Convert from degrees to radians
            for (int i = 0; i < 6; i++) {
                current_state_.positions[i] = current_state_.positions[i];
                current_state_.velocities[i] = current_state_.velocities[i];
            }
            state_updated_ = true;
        }

        if (!warm_start_complete_) {
            return;
        }

        if (solver_->isTrajectoryComplete()) {
            return;
        }

        optimization_in_progress_ = true;

        full_state_.clear();  // Clear before inserting new state
        full_state_.insert(full_state_.end(), 
            current_state_.positions.begin(), 
            current_state_.positions.end());
        full_state_.insert(full_state_.end(), 
            current_state_.velocities.begin(), 
            current_state_.velocities.end());

        // Calculate elapsed time based on use_sim_time_ setting
        double elapsed_time;
        if (use_sim_time_) {
            SimTime current_time(msg->header.stamp);
            SimTime start_time(trajectory_start_stamp_);
            elapsed_time = (current_time - start_time).seconds();
        } else {
            auto now = Clock::now();
            Duration elapsed = now - trajectory_start_time_;
            elapsed_time = elapsed.count();
        }
        
        // Shift trajectory and run optimization
        solver_->shiftTrajectory(full_state_, elapsed_time);
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
        traj_msg.knot_points = solver_->numKnotPoints(); 
        traj_msg.dt = timestep_.count();  // Convert Duration to double

        // Get optimized trajectory from solver
        const auto [traj_data, traj_size] = solver_->getOptimizedTrajectory();
        
        const int stride = solver_->stateSize() + solver_->controlSize();
        
        for (int i = 0; i < traj_msg.knot_points; i++) {
            for (size_t j = 0; j < 6; ++j) {
                traj_msg.points[i].positions[j] = traj_data[i * stride + j];// * 180.0 / M_PI;
                traj_msg.points[i].velocities[j] = traj_data[i * stride + solver_->stateSize()/2 + j];// * 180.0 / M_PI;
                traj_msg.points[i].torques[j] = traj_data[i * stride + solver_->stateSize() + j];
            }
        }

        traj_pub_->publish(traj_msg);
    }

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

    msgs::JointTrajectory traj_msg_;
    std::vector<float> full_state_;

    bool use_sim_time_;
    builtin_interfaces::msg::Time trajectory_start_stamp_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajoptNode>(argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}