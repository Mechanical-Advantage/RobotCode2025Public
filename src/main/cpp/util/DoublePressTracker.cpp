// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/DoublePressTracker.h"

namespace util {

frc2::command::button::Trigger
DoublePressTracker::DoublePress(frc2::command::button::Trigger baseTrigger) {
  DoublePressTracker tracker(baseTrigger);
  return frc2::command::button::Trigger([&tracker]() { return tracker.Get(); });
}

DoublePressTracker::DoublePressTracker(
    frc2::command::button::Trigger baseTrigger)
    : trigger(baseTrigger) {}

bool DoublePressTracker::Get() {
  bool pressed = trigger.Get();
  switch (state) {
  case DoublePressState::IDLE:
    if (pressed) {
      state = DoublePressState::FIRST_PRESS;
      resetTimer.Reset();
      resetTimer.Start();
    }
    break;
  case DoublePressState::FIRST_PRESS:
    if (!pressed) {
      if (resetTimer.HasElapsed(maxLengthSecs)) {
        Reset();
      } else {
        state = DoublePressState::FIRST_RELEASE;
      }
    }
    break;
  case DoublePressState::FIRST_RELEASE:
    if (pressed) {
      state = DoublePressState::SECOND_PRESS;
    } else if (resetTimer.HasElapsed(maxLengthSecs)) {
      Reset();
    }
    break;
  case DoublePressState::SECOND_PRESS:
    if (!pressed) {
      Reset();
    }
    break;
  }
  return state == DoublePressState::SECOND_PRESS;
}

void DoublePressTracker::Reset() {
  state = DoublePressState::IDLE;
  resetTimer.Stop();
}

} // namespace util