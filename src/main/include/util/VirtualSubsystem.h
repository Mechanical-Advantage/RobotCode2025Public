// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <memory>
#include <vector>

namespace frc2025::util {

class VirtualSubsystem {
public:
  VirtualSubsystem();
  virtual ~VirtualSubsystem() = default;

  static void periodicAll();

  virtual void periodic() = 0;

private:
  static std::vector<std::unique_ptr<VirtualSubsystem>> subsystems;
};

} // namespace frc2025::util