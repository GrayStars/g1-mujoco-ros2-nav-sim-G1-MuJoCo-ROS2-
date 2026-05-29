#pragma once

#include "unitree/dds_wrapper/robots/g1/g1.h"

using LowCmd_t = unitree::robot::g1::publisher::LowCmd;
using LowState_t = unitree::robot::g1::subscription::LowState;

using LowStateMsg_t = unitree_hg::msg::dds_::LowState_;
using SportModeStateMsg_t = unitree_hg::msg::dds_::SportModeState_;
