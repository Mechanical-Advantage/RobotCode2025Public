// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/CanivoreReader.h"
#include "Constants.h"

namespace util {

CanivoreReader::CanivoreReader(const std::string &canBusName)
    : canBus(canBusName), notifier([this]() { UpdateStatus(); }) {
  notifier.StartPeriodic(Constants::loopPeriodSecs);
}

std::optional<phoenix6::CANBus::CANBusStatus> CanivoreReader::GetStatus() {
  std::lock_guard<std::mutex> lock(statusMutex);
  return status;
}

void CanivoreReader::UpdateStatus() {
  auto statusTemp = canBus.GetStatus();
  {
    std::lock_guard<std::mutex> lock(statusMutex);
    status = statusTemp;
  }
}

} // namespace util