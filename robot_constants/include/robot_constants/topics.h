#pragma once

#include <string>

namespace robot_constants {

namespace topics {

const std::string kLeftWheelSetpoint = "/left_wheel_setpoint";
const std::string kRightWheelSetpoint = "/right_wheel_setpoint";

const std::string kLeftWheelCurPos = "/left_wheel_cur_pos";
const std::string kRightWheelCurPos = "/right_wheel_cur_pos";
const std::string kLeftWheelCurAngVel = "/left_wheel_cur_ang_vel";
const std::string kRightWheelCurAngVel = "/right_wheel_cur_ang_vel";

}   // namespace topics

}   // namespace robot_constants
