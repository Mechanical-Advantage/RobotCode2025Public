// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "PhoenixUtil.h"

namespace frc2025::util {

std::vector<ctre::phoenix6::BaseStatusSignal *> PhoenixUtil::canivoreSignals;
std::vector<ctre::phoenix6::BaseStatusSignal *> PhoenixUtil::rioSignals;

void PhoenixUtil::tryUntilOk(
    int maxAttempts, std::function<ctre::phoenix6::StatusCode()> command) {
  for (int i = 0; i < maxAttempts; i++) {
    ctre::phoenix6::StatusCode error = command();
    if (error.IsOK())
      break;
  }
}

void PhoenixUtil::registerSignals(
    bool canivore, std::vector<ctre::phoenix6::BaseStatusSignal *> signals) {
  if (canivore) {
    std::vector<ctre::phoenix6::BaseStatusSignal *> newSignals;
    newSignals.reserve(canivoreSignals.size() + signals.size());
    newSignals.insert(newSignals.end(), canivoreSignals.begin(),
                      canivoreSignals.end());
    newSignals.insert(newSignals.end(), signals.begin(), signals.end());
    canivoreSignals = newSignals;
  } else {
    std::vector<ctre::phoenix6::BaseStatusSignal *> newSignals;
    newSignals.reserve(rioSignals.size() + signals.size());
    newSignals.insert(newSignals.end(), rioSignals.begin(), rioSignals.end());
    newSignals.insert(newSignals.end(), signals.begin(), signals.end());
    rioSignals = newSignals;
  }
}

void PhoenixUtil::refreshAll() {
  if (!canivoreSignals.empty()) {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(canivoreSignals);
  }
  if (!rioSignals.empty()) {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(rioSignals);
  }
}

} // namespace frc2025::util