#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "h2/h2_loco_client.hpp"

using namespace std::chrono_literals;

struct TestOption
{
    std::string name;
    int id;
};

static const std::vector<TestOption> option_list = {
    {"damp", 0},
    {"start", 1},
    {"stand_up", 2},
    {"zero_torque", 3},
    {"stop_move", 4},
    {"get_fsm_id", 5},
    {"get_fsm_mode", 6},
    {"set_fsm_id", 7},
    {"set_velocity", 8},
    {"move", 9},
    {"switch_move_mode", 10},
    {"set_speed_mode", 11},
};

static int ConvertToInt(const std::string &str)
{
    try
    {
        return std::stoi(str);
    }
    catch (...)
    {
        return -1;
    }
}

class UserInterface
{
public:
    void terminalHandle()
    {
        std::string input;
        std::getline(std::cin, input);

        if (input == "list")
        {
            for (const auto &option : option_list)
            {
                std::cout << option.name << ", id: " << option.id << std::endl;
            }
            return;
        }

        const int id = ConvertToInt(input);
        if (id < 0)
        {
            std::cout << "Unknown command. Input \"list\" or an option id." << std::endl;
            return;
        }

        for (const auto &option : option_list)
        {
            if (id == option.id)
            {
                test_option_->id = option.id;
                test_option_->name = option.name;
                std::cout << "Test: " << test_option_->name << ", test_id: " << test_option_->id << std::endl;
                return;
            }
        }

        std::cout << "Unknown id. Input \"list\" to list all options." << std::endl;
    }

    TestOption *test_option_{nullptr};
};

class H2LocoClientNode : public rclcpp::Node
{
public:
    H2LocoClientNode() : Node("h2_loco_client_node"), client_(this)
    {
        test_option_.id = 0;
        test_option_.name = "damp";
        ui_.test_option_ = &test_option_;

        t1_ = std::thread([this]
                          {
                              std::this_thread::sleep_for(500ms);
                              Run();
                          });
    }

    ~H2LocoClientNode() override
    {
        if (t1_.joinable())
            t1_.join();
    }

private:
    void Run()
    {
        std::cout << "Input \"list\" to list all test option ..." << std::endl;
        while (rclcpp::ok())
        {
            ui_.terminalHandle();

            int32_t ret = 0;
            if (test_option_.name == "damp")
            {
                ret = client_.Damp();
            }
            else if (test_option_.name == "start")
            {
                ret = client_.Start();
            }
            else if (test_option_.name == "stand_up")
            {
                ret = client_.StandUp();
            }
            else if (test_option_.name == "zero_torque")
            {
                ret = client_.ZeroTorque();
            }
            else if (test_option_.name == "stop_move")
            {
                ret = client_.StopMove();
            }
            else if (test_option_.name == "get_fsm_id")
            {
                int fsm_id = -1;
                ret = client_.GetFsmId(fsm_id);
                if (ret == 0)
                    std::cout << "fsm_id: " << fsm_id << std::endl;
            }
            else if (test_option_.name == "get_fsm_mode")
            {
                int fsm_mode = -1;
                ret = client_.GetFsmMode(fsm_mode);
                if (ret == 0)
                    std::cout << "fsm_mode: " << fsm_mode << std::endl;
            }
            else if (test_option_.name == "set_fsm_id")
            {
                ret = client_.SetFsmId(4);
            }
            else if (test_option_.name == "set_velocity")
            {
                ret = client_.SetVelocity(0.0F, 0.0F, 1.0F, 4.0F);
            }
            else if (test_option_.name == "move")
            {
                ret = client_.Move(0.0F, 0.0F, 2.0F);
            }
            else if (test_option_.name == "switch_move_mode")
            {
                ret = client_.SwitchMoveMode(true);
            }
            else if (test_option_.name == "set_speed_mode")
            {
                ret = client_.SetSpeedMode(1);
            }
            else
            {
                std::cout << "Unknown option." << std::endl;
                continue;
            }

            if (ret != 0)
            {
                std::cout << "Call failed, ret: " << ret << std::endl;
            }
            else
            {
                std::cout << "Call OK, ret: " << ret << std::endl;
            }
        }
    }

    unitree::robot::h2::H2LocoClient client_;
    TestOption test_option_;
    UserInterface ui_;
    std::thread t1_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<H2LocoClientNode>();
    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(200ms);
    }
    rclcpp::shutdown();
    return 0;
}

