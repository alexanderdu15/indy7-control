#include "indydcp3.h"
#include "csv_utils.h"
#include <vector>
#include <thread>
#include <chrono>
#include <iostream>
 
class Indy7JointController {
public:
    Indy7JointController(const std::string& robot_ip, int index = 0, double timestep = 1)
        : indyDCP3(robot_ip, index), _timestep(timestep) { 
            Nrmk::IndyFramework::ControlData control_data;
            bool is_success = indyDCP3.get_robot_data(control_data);
            if (is_success) {
                _cobotDOF = control_data.q().size();
            }

            start_teleop();
            std::this_thread::sleep_for(std::chrono::seconds(1)); //wait for the robot to be ready
        }
    
    ~Indy7JointController() {
        stop_teleop();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    void sendTrajectory(const std::vector<std::vector<float>>& trajectory) {
        if (trajectory.empty()) {
            std::cerr << "Empty trajectory provided." << std::endl;
            return;
        }

        if (trajectory[0].size() != _cobotDOF) { // make sure waypoint size matches the robot's DOF
            std::cerr << "Invalid joint positions size. Expected " << _cobotDOF 
                      << " but got " << trajectory[0].size() << "." << std::endl;
            return;
        }

        move_joints_teleop(trajectory[0]);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        std::cout << "Starting trajectory execution..." << std::endl;

        auto start_time = std::chrono::high_resolution_clock::now();

        // Execute each waypoint in the trajectory
        for (const auto& waypoint : trajectory) {
            move_joints_teleop(waypoint);
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(_timestep)));
        }
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        double execution_time_seconds = duration.count() / 1000.0;
        double control_rate = trajectory.size() / execution_time_seconds;
        
        std::cout << "Trajectory execution completed." << std::endl;
        std::cout << "Execution time: " << execution_time_seconds << " seconds" << std::endl;
        std::cout << "Number of waypoints: " << trajectory.size() << std::endl;
        std::cout << "Control rate: " << control_rate << " Hz" << std::endl;
    }

    void start_teleop() {
        bool success = indyDCP3.start_teleop(TeleMethod::TELE_JOINT_ABSOLUTE);
        if (!success) {
            std::cerr << "Failed to start teleoperation." << std::endl;
        }
    }

    void stop_teleop() {
        bool success = indyDCP3.stop_teleop();
        if (success) {
            std::cout << "Teleoperation stopped successfully." << std::endl;
        } else {
            std::cerr << "Failed to stop teleoperation." << std::endl;
        }
    }

    void move_joints_teleop(const std::vector<float>& q) {
        bool success = indyDCP3.movetelej(q, 0.5, 1.0, TeleMethod::TELE_JOINT_ABSOLUTE);
        if (!success) {
            std::cerr << "Failed to move joint positions in teleoperation mode." << std::endl;
        }
    }

    void move_joints(const std::vector<float>& q) {
        bool success = indyDCP3.movej(q, JointBaseType::ABSOLUTE_JOINT, BlendingType_Type::BlendingType_Type_NONE);
        if (!success) {
            std::cerr << "MoveJ command failed." << std::endl;
        }
    }



private:
    IndyDCP3 indyDCP3;
    int _cobotDOF;
    int _timestep;
};

// Example usage
int main() {
    // Initialize JointControl with the robot's IP address
    Indy7JointController jointController("160.39.102.52");

    // read trajectory from csv file
    std::vector<std::vector<float>> trajectory = readCsvToVectorOfVectors<float>("../examples/trajectories/simple_trajectory.csv");

    // Send the trajectory to the robot
    jointController.sendTrajectory(trajectory);

    return 0;
}

