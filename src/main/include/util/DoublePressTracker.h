// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/button/Trigger.h>

namespace util {

/** Tracker that activates only when a button is pressed twice quickly. */
class DoublePressTracker {
public:
  static constexpr double maxLengthSecs =
      0.4; // How long after the first press does the second need to occur?

  static frc2::command::button::Trigger
  DoublePress(frc2::command::button::Trigger baseTrigger);

private:
  enum class DoublePressState {
    IDLE,
    FIRST_PRESS,
    FIRST_RELEASE,
    SECOND_PRESS
  };

  DoublePressTracker(frc2::command::button::Trigger baseTrigger);
  bool Get();
  void Reset();

  frc2::command::button::Trigger trigger;
  frc::Timer resetTimer;
  DoublePressState state = DoublePressState::IDLE;
};

} // namespace util