// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <string>

#include <frc/Timer.h>

namespace util {

/** Utility class for logging code execution times. */
class LoggedTracer {
public:
  /** Reset the clock. */
  static void Reset();

  /** Save the time elapsed since the last reset or record. */
  static void Record(const std::string &epochName);

private:
  LoggedTracer() = delete; // Prevent instantiation
  static double startTime;
};

} // namespace util