// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/GenericHID.h>
#include <frc2/command/button/Trigger.h>

namespace util {

/** Interface for physical override switches on operator console. */
class OverrideSwitches {
public:
  enum class MultiDirectionSwitchState { LEFT, NEUTRAL, RIGHT };

  OverrideSwitches(int port);

  /** Returns whether the controller is connected. */
  bool IsConnected() const;

  /** Gets the state of a driver-side switch (0-2 from left to right). */
  bool GetDriverSwitch(int index) const;

  /** Gets the state of an operator-side switch (0-4 from left to right). */
  bool GetOperatorSwitch(int index) const;

  /** Gets the state of the multi-directional switch. */
  MultiDirectionSwitchState GetMultiDirectionSwitch() const;

  /** Returns a trigger for a driver-side switch (0-2 from left to right). */
  frc2::command::button::Trigger DriverSwitch(int index) const;

  /** Returns a trigger for an operator-side switch (0-4 from left to right). */
  frc2::command::button::Trigger OperatorSwitch(int index) const;

  /** Returns a trigger for when the multi-directional switch is pushed to the
   * left. */
  frc2::command::button::Trigger MultiDirectionSwitchLeft() const;

  /** Returns a trigger for when the multi-directional switch is pushed to the
   * right. */
  frc2::command::button::Trigger MultiDirectionSwitchRight() const;

private:
  frc::GenericHID joystick;
};

} // namespace util