// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "TriggerUtil.h"

namespace frc2025::util {

void TriggerUtil::whileTrueContinuous(frc2::Trigger &trigger,
                                      frc2::Command *command) {
  frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop().Bind(
      [&trigger, command]() {
        static bool m_pressedLast = trigger.Get();
        bool pressed = trigger.Get();

        if (pressed) {
          command->Schedule();
        } else if (m_pressedLast) {
          command->Cancel();
        }

        m_pressedLast = pressed;
      });
}

} // namespace frc2025::util