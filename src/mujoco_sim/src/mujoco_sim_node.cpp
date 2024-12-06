#include "rclcpp/rclcpp.hpp"
#include "indy7_msgs/msg/joint_state.hpp"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <memory>
#include <mutex>
#include <chrono>
#include <string>
#include <vector>

namespace msgs = indy7_msgs::msg;

using namespace std::chrono_literals;

float camera_azimuth = 90.0;
float camera_elevation = -20.0;
float camera_distance = 2.0;
float camera_sensitivity = 0.5;

bool mouse_drag = false;
double last_x = 0.0, last_y = 0.0;

void glfw_error_callback(int error, const char *description)
{
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

void keyboard_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if (action == GLFW_PRESS)
        {
            mouse_drag = true;
        }
        else if (action == GLFW_RELEASE)
        {
            mouse_drag = false;
        }
    }
}

void cursor_position_callback(GLFWwindow *window, double xpos, double ypos)
{
    if (mouse_drag)
    {
        camera_azimuth -= (xpos - last_x) * camera_sensitivity;
        camera_elevation -= (ypos - last_y) * camera_sensitivity;

        if (camera_elevation > 90.0)
            camera_elevation = 90.0;
        if (camera_elevation < -90.0)
            camera_elevation = -90.0;
    }

    last_x = xpos;
    last_y = ypos;
}

class MujocoSimNode : public rclcpp::Node {
public:
    MujocoSimNode() : Node("mujoco_sim_node") {
        RCLCPP_INFO(get_logger(), "Initializing MuJoCo simulation node");
        if (!glfwInit())
        {
            std::cerr << "Could not initialize GLFW" << std::endl;
        }
        glfwSetErrorCallback(glfw_error_callback);

        window_ = glfwCreateWindow(1200, 900, "MuJoCo Simulator", nullptr, nullptr);
        if (!window_)
        {
            std::cerr << "Could not create GLFW window" << std::endl;
            glfwTerminate();
        }
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);

        glfwSetMouseButtonCallback(window_, mouse_button_callback);
        glfwSetCursorPosCallback(window_, cursor_position_callback);
        glfwSetKeyCallback(window_, keyboard_callback);

        RCLCPP_INFO(get_logger(), "Loading MuJoCo model");
        std::string model_path = "/home/a2rlab/Indy7/temp/indy7-control/src/mujoco_sim/models/indy7.xml";
        
        // Load MuJoCo model
        char error[1000];
        model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
        if (!model_) {
            RCLCPP_ERROR(get_logger(), "Failed to load model: %s", error);
            throw std::runtime_error("Model loading failed");
        }

        model_->opt.gravity[0] = 0;
        model_->opt.gravity[1] = 0;
        model_->opt.gravity[2] = 0;

        // Initialize MuJoCo data
        data_ = mj_makeData(model_);

        // Set initial joint positions (in radians)
        std::vector<double> initial_positions = {1, 0.0, -1.62, 0.0, -0.5, 0.0};
        for (size_t i = 0; i < initial_positions.size(); ++i) {
            data_->qpos[i] = initial_positions[i];
        }
        
        // Reset the simulation to apply initial state
        mj_forward(model_, data_);

        //wait for 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));

        mjv_defaultScene(&scene_);
        mjv_defaultCamera(&camera_);
        mjv_defaultOption(&options_);
        mjr_defaultContext(&context_);

        mjv_makeScene(model_, &scene_, 2000);
        mjr_makeContext(model_, &context_, mjFONTSCALE_150);

        camera_.type = mjCAMERA_FREE;
        camera_.distance = camera_distance;
        camera_.azimuth = camera_azimuth;
        camera_.elevation = camera_elevation;
        
        // Set position control mode by setting the gains
        for (int i = 0; i < model_->nu; i++) {
            model_->actuator_gainprm[i * 3] = 100.0;  
            model_->actuator_gainprm[i * 3 + 1] = 10.0; 
        }
        
        // Create publishers and subscribers (change topic names to reflect position control)
        state_pub_ = create_publisher<msgs::JointState>("joint_states", 10);
        position_sub_ = create_subscription<msgs::JointState>(
            "joint_commands", 1,
            std::bind(&MujocoSimNode::commandCallback, this, std::placeholders::_1)
        );

        // Initialize simulation time tracking
        sim_time_ = 0.0;
        sim_timestep_ = 0.01;  
        
        // Create simulation timer at 1kHz
        sim_timer_ = create_wall_timer(
            std::chrono::duration<double>(sim_timestep_),
            std::bind(&MujocoSimNode::simulationStep, this)
        );

        model_->opt.timestep = sim_timestep_;

        RCLCPP_INFO(get_logger(), "MuJoCo simulation node initialized with sim timestep: %f",
                    sim_timestep_);

        // Set use_sim_time parameter for this node
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        
        // Initialize simulation clock
        sim_clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
        current_sim_time_ = sim_clock_->now();
    }

    ~MujocoSimNode() {
        if (data_) mj_deleteData(data_);
        if (model_) mj_deleteModel(model_);
        mj_deleteData(data_);
        mj_deleteModel(model_);
        mjv_freeScene(&scene_);
        mjr_freeContext(&context_);
        glfwDestroyWindow(window_);
        glfwTerminate();
    }

private:
    void commandCallback(const msgs::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        // Apply received position commands to the simulation
        RCLCPP_INFO(get_logger(), "Received position command: %f %f %f %f %f %f",
                    msg->positions[0], msg->positions[1], msg->positions[2],
                    msg->positions[3], msg->positions[4], msg->positions[5]);
        
        // Set position targets
        for (size_t i = 0; i < msg->positions.size(); ++i) {
            data_->ctrl[i] = msg->positions[i];
        }
    }

    void simulationStep() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Step the simulation
        mj_step(model_, data_);
        sim_time_ += sim_timestep_;
        
        // Update simulation time
        current_sim_time_ = current_sim_time_ + rclcpp::Duration::from_seconds(sim_timestep_);

        // Publish state at simulation rate
        auto state_msg = std::make_unique<msgs::JointState>();
        state_msg->header.stamp = current_sim_time_;  // Use simulation time
        
        // Fill joint positions, velocities, and efforts
        for (int i = 0; i < model_->nv; ++i) {
            state_msg->positions[i] = data_->qpos[i];
            state_msg->velocities[i] = data_->qvel[i]; 
            state_msg->torques[i] = data_->ctrl[i];
        }

        state_pub_->publish(std::move(state_msg));

        camera_.azimuth = camera_azimuth;
        camera_.elevation = camera_elevation;
        camera_.distance = camera_distance;

        int width, height;
        glfwGetFramebufferSize(window_, &width, &height);

        mjrRect viewport = {0, 0, width, height};
        mjv_updateScene(model_, data_, &options_, nullptr, &camera_, mjCAT_ALL, &scene_);
        mjr_render(viewport, &scene_, &context_);

        glfwSwapBuffers(window_);
        glfwPollEvents();
    }

    mjModel* model_;
    mjData* data_;
    std::mutex mutex_;

    mjvScene scene_;
    mjvCamera camera_;
    mjvOption options_;
    mjrContext context_;
    GLFWwindow *window_;
    
    rclcpp::Publisher<msgs::JointState>::SharedPtr state_pub_;
    rclcpp::Subscription<msgs::JointState>::SharedPtr position_sub_;
    rclcpp::TimerBase::SharedPtr sim_timer_;
    double sim_time_;
    double sim_timestep_; 
    rclcpp::Time current_sim_time_;
    rclcpp::Clock::SharedPtr sim_clock_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MujocoSimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
