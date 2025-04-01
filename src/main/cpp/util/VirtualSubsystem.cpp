// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "VirtualSubsystem.h"

namespace frc2025::util {

std::vector<std::unique_ptr<VirtualSubsystem>> VirtualSubsystem::subsystems;

VirtualSubsystem::VirtualSubsystem() { subsystems.emplace_back(this); }

void VirtualSubsystem::periodicAll() {
  for (const auto &subsystem : subsystems) {
    subsystem->periodic();
  }
}

} // namespace frc2025::util