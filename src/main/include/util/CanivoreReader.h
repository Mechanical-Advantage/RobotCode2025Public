// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <optional>
#include <string>

#include <frc/Notifier.h>
#include <frc/Timer.h>
#include <phoenix6/CANBus.hpp>

#include "Constants.h"

namespace util {

class CanivoreReader {
public:
  CanivoreReader(const std::string &canBusName);
  std::optional<phoenix6::CANBus::CANBusStatus> GetStatus();

private:
  phoenix6::CANBus canBus;
  frc::Notifier notifier;
  std::optional<phoenix6::CANBus::CANBusStatus> status;

  void UpdateStatus();
};

} // namespace util