// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/button/Trigger.h>

namespace frc2025::util {

class TriggerUtil {
public:
  /**
   * Constantly starts the given command while the button is held.
   *
   * <p>{@link Command#schedule()} will be called repeatedly while the trigger
   * is active, and will be canceled when the trigger becomes inactive.
   *
   * @param command the command to start
   * @return this trigger, so calls can be chained
   */
  static void whileTrueContinuous(frc2::Trigger &trigger,
                                  frc2::Command *command);

private:
  TriggerUtil() = delete;
};

} // namespace frc2025::util