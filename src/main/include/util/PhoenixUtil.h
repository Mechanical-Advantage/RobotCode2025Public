// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <ctre/phoenix6/BaseStatusSignal.hpp>
#include <ctre/phoenix6/StatusCode.hpp>
#include <functional>
#include <vector>

namespace frc2025::util {

class PhoenixUtil {
public:
  /** Attempts to run the command until no error is produced. */
  static void tryUntilOk(int maxAttempts,
                         std::function<ctre::phoenix6::StatusCode()> command);

  /** Registers a set of signals for synchronized refresh. */
  static void
  registerSignals(bool canivore,
                  std::vector<ctre::phoenix6::BaseStatusSignal *> signals);

  /** Refresh all registered signals. */
  static void refreshAll();

private:
  static std::vector<ctre::phoenix6::BaseStatusSignal *> canivoreSignals;
  static std::vector<ctre::phoenix6::BaseStatusSignal *> rioSignals;
};

} // namespace frc2025::util