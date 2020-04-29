#pragma once

#include <string>

namespace robot_constants {

namespace topics {

const std::string kLeftWheelControlEffort = "/left_wheel/control_effort";
const std::string kLeftWheelSetpoint = "/left_wheel/setpoint";
const std::string kLeftWheelPosition = "/left_wheel/pos";
const std::string kLeftWheelVelocity = "/left_wheel/vel";

const std::string kRightWheelControlEffort = "/right_wheel/control_effort";
const std::string kRightWheelSetpoint = "/right_wheel/setpoint";
const std::string kRightWheelPosition = "/right_wheel/pos";
const std::string kRightWheelVelocity = "/right_wheel/vel";

}   // namespace topics

}   // namespace robot_constants
