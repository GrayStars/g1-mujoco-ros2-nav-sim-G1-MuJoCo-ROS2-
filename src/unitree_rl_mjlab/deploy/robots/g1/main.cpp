#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Mimic.h"
#include "input/velocity_command_source.h"
#include <memory>
#include <string>
#include <unitree/idl/ros2/Twist_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = std::make_shared<Keyboard>();
std::shared_ptr<unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::Twist_>> cmd_vel_sub = nullptr;

std::string ros_topic_to_dds_topic(const std::string &topic)
{
    if(topic.rfind("rt/", 0) == 0)
    {
        return topic;
    }
    if(!topic.empty() && topic[0] == '/')
    {
        return "rt" + topic;
    }
    return "rt/" + topic;
}

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);
    if(vm.count("keyboard") && vm.count("cmd_vel"))
    {
        spdlog::critical("Please use only one input mode: --keyboard or --cmd_vel.");
        exit(-1);
    }

    input::VelocityCommandSource::Mode command_mode = input::VelocityCommandSource::Mode::Joystick;
    if(vm.count("keyboard"))
    {
        command_mode = input::VelocityCommandSource::Mode::Keyboard;
    }
    else if(vm.count("cmd_vel"))
    {
        command_mode = input::VelocityCommandSource::Mode::CmdVel;
    }
    input::VelocityCommandSource::setMode(command_mode);
    const bool auto_start = command_mode != input::VelocityCommandSource::Mode::Joystick;

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     G1-29dof Controller \n";

    // Unitree DDS Config
    const auto domain_id = vm["domain"].as<int>();
    unitree::robot::ChannelFactory::Instance()->Init(domain_id, vm["network"].as<std::string>());

    if(command_mode == input::VelocityCommandSource::Mode::CmdVel)
    {
        const auto cmd_vel_topic = ros_topic_to_dds_topic(vm["cmd_vel_topic"].as<std::string>());
        cmd_vel_sub = std::make_shared<unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::Twist_>>(
            cmd_vel_topic,
            [](const void *message)
            {
                const auto *msg = static_cast<const geometry_msgs::msg::dds_::Twist_*>(message);
                input::VelocityCommandSource::setCommand(
                    static_cast<float>(msg->linear().x()),
                    static_cast<float>(msg->linear().y()),
                    static_cast<float>(msg->angular().z())
                );
            },
            10
        );
        cmd_vel_sub->InitChannel();
        std::cout << "ROS2 cmd_vel mode enabled. Listening DDS topic: " << cmd_vel_topic << "\n";
    }

    init_fsm_state();

    FSMState::lowcmd->msg_.mode_machine() = 5; // 29dof
    if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate)) {
        spdlog::critical("Unmatched robot type.");
        exit(-1);
    }

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"], auto_start);
    fsm->start();

    if(command_mode == input::VelocityCommandSource::Mode::Keyboard)
    {
        std::cout << "Keyboard mode enabled: Passive -> FixStand -> Velocity.\n";
        std::cout << "Keyboard velocity commands: w/s vx, a/d vy, q/e omega_z, space zero.\n";
    }
    else if(command_mode == input::VelocityCommandSource::Mode::CmdVel)
    {
        std::cout << "cmd_vel mode enabled: Passive -> FixStand -> Velocity.\n";
        std::cout << "ROS2 /cmd_vel mapping: linear.x vx, linear.y vy, angular.z omega_z.\n";
    }
    else
    {
        std::cout << "Press [L2 + Up] to enter FixStand mode.\n";
        std::cout << "And then press [R2 + A] to start controlling the robot.\n";
        std::cout << "And then press [R1 + A/B/Y/X] to control the robot dance.\n";
    }

    while (true)
    {
        sleep(1);
    }
    
    return 0;
}
