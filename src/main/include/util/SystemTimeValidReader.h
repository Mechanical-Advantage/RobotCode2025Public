// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <atomic>
#include <frc/Notifier.h>
#include <frc/RobotController.h>
#include <frc/Timer.h>
#include <mutex>

namespace frc2025::util {

class SystemTimeValidReader {
public:
  static void start();
  static bool isValid();

private:
  static std::unique_ptr<frc::Notifier> notifier;
  static std::atomic<bool> ready;
  static std::mutex mutex;
};

} // namespace frc2025::util