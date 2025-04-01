// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "SystemTimeValidReader.h"

namespace frc2025::util {

std::unique_ptr<frc::Notifier> SystemTimeValidReader::notifier = nullptr;
std::atomic<bool> SystemTimeValidReader::ready = false;
std::mutex SystemTimeValidReader::mutex;

void SystemTimeValidReader::start() {
  if (notifier != nullptr)
    return;
  notifier = std::make_unique<frc::Notifier>([]() {
    bool readyNew = frc::RobotController::IsSystemTimeValid();
    {
      std::lock_guard<std::mutex> lock(mutex);
      ready = readyNew;
    }
  });
  notifier->StartPeriodic(3.0);
}

bool SystemTimeValidReader::isValid() {
  std::lock_guard<std::mutex> lock(mutex);
  return ready;
}

} // namespace frc2025::util