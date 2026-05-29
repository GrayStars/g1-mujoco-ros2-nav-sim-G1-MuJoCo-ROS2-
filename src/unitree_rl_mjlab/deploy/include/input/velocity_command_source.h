#pragma once

#include <algorithm>
#include <array>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>

namespace input
{

class VelocityCommandSource
{
public:
    enum class Mode
    {
        Joystick,
        Keyboard,
        CmdVel,
    };

    static void setMode(Mode new_mode)
    {
        std::lock_guard<std::mutex> lock(mutex());
        mode() = new_mode;
        command() = {0.0f, 0.0f, 0.0f};
        if (new_mode != Mode::Joystick) {
            printCommandLocked();
        }
    }

    static Mode getMode()
    {
        std::lock_guard<std::mutex> lock(mutex());
        return mode();
    }

    static bool usesExternalCommand()
    {
        std::lock_guard<std::mutex> lock(mutex());
        return mode() != Mode::Joystick;
    }

    static void updateKeyboard(const std::string &key, bool on_pressed)
    {
        std::lock_guard<std::mutex> lock(mutex());
        if (mode() != Mode::Keyboard || !on_pressed) {
            return;
        }

        bool changed = true;
        auto &cmd = command();
        if (key == "w") {
            cmd[0] += step;
        } else if (key == "s") {
            cmd[0] -= step;
        } else if (key == "a") {
            cmd[1] += step;
        } else if (key == "d") {
            cmd[1] -= step;
        } else if (key == "q") {
            cmd[2] += step;
        } else if (key == "e") {
            cmd[2] -= step;
        } else if (key == " ") {
            cmd = {0.0f, 0.0f, 0.0f};
        } else {
            changed = false;
        }

        if (changed) {
            printCommandLocked();
        }
    }

    static void setCommand(float vx, float vy, float omega_z)
    {
        std::lock_guard<std::mutex> lock(mutex());
        if (mode() == Mode::Joystick) {
            return;
        }
        command() = {vx, vy, omega_z};
        printCommandLocked();
    }

    static std::array<float, 3> clampedCommand(
        float vx_min, float vx_max,
        float vy_min, float vy_max,
        float omega_min, float omega_max)
    {
        std::lock_guard<std::mutex> lock(mutex());
        auto cmd = command();
        cmd[0] = std::clamp(cmd[0], vx_min, vx_max);
        cmd[1] = std::clamp(cmd[1], vy_min, vy_max);
        cmd[2] = std::clamp(cmd[2], omega_min, omega_max);
        command() = cmd;
        return cmd;
    }

private:
    static constexpr float step = 0.1f;

    static Mode &mode()
    {
        static Mode current = Mode::Joystick;
        return current;
    }

    static std::array<float, 3> &command()
    {
        static std::array<float, 3> cmd = {0.0f, 0.0f, 0.0f};
        return cmd;
    }

    static std::mutex &mutex()
    {
        static std::mutex mtx;
        return mtx;
    }

    static void printCommandLocked()
    {
        const auto &cmd = command();
        const char *source = "joystick";
        if (mode() == Mode::Keyboard) {
            source = "keyboard";
        } else if (mode() == Mode::CmdVel) {
            source = "cmd_vel";
        }
        std::cout << std::fixed << std::setprecision(2)
                  << "[" << source << "] vx=" << cmd[0]
                  << " vy=" << cmd[1]
                  << " omega_z=" << cmd[2] << std::endl;
    }
};

} // namespace input
