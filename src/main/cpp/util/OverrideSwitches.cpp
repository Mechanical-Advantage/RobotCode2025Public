// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "OverrideSwitches.h"
#include <stdexcept>

OverrideSwitches::OverrideSwitches(int port) : joystick(port) {}

bool OverrideSwitches::isConnected() { return joystick.IsConnected(); }

bool OverrideSwitches::getDriverSwitch(int index) {
  if (index < 0 || index > 2) {
    throw std::runtime_error("Invalid driver override index " +
                             std::to_string(index) + ". Must be 0-2.");
  }
  return joystick.GetRawButton(index + 1);
}

bool OverrideSwitches::getOperatorSwitch(int index) {
  if (index < 0 || index > 4) {
    throw std::runtime_error("Invalid operator override index " +
                             std::to_string(index) + ". Must be 0-4.");
  }
  return joystick.GetRawButton(index + 8);
}

OverrideSwitches::MultiDirectionSwitchState
OverrideSwitches::getMultiDirectionSwitch() {
  if (joystick.GetRawButton(4)) {
    return MultiDirectionSwitchState::LEFT;
  } else if (joystick.GetRawButton(5)) {
    return MultiDirectionSwitchState::RIGHT;
  } else {
    return MultiDirectionSwitchState::NEUTRAL;
  }
}

frc2::Trigger OverrideSwitches::driverSwitch(int index) {
  return frc2::Trigger([this, index]() { return getDriverSwitch(index); });
}

frc2::Trigger OverrideSwitches::operatorSwitch(int index) {
  return frc2::Trigger([this, index]() { return getOperatorSwitch(index); });
}

frc2::Trigger OverrideSwitches::multiDirectionSwitchLeft() {
  return frc2::Trigger([this]() {
    return getMultiDirectionSwitch() == MultiDirectionSwitchState::LEFT;
  });
}

frc2::Trigger OverrideSwitches::multiDirectionSwitchRight() {
  return frc2::Trigger([this]() {
    return getMultiDirectionSwitch() == MultiDirectionSwitchState::RIGHT;
  });
}