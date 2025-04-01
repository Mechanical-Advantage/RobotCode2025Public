// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/LoggedTracer.h"

#include <frc/Timer.h>
#include <wpi/Logger.h>

namespace util {

double LoggedTracer::startTime = -1.0;

void LoggedTracer::Reset() { startTime = frc::Timer::GetFPGATimestamp(); }

void LoggedTracer::Record(const std::string &epochName) {
  double now = frc::Timer::GetFPGATimestamp();
  wpi::log::DataLog::RecordOutput("LoggedTracer/" + epochName + "MS",
                                  (now - startTime) * 1000.0);
  startTime = now;
}

} // namespace util