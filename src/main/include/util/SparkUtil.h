// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <rev/CANSparkMax.h>
#include <vector>

namespace frc2025::util {

class SparkUtil {
public:
  /** Stores whether any error was has been detected by other utility methods.
   */
  static bool sparkStickyFault;

  /** Processes a value from a Spark only if the value is valid. */
  static void ifOk(rev::CANSparkMax &spark, std::function<double()> supplier,
                   std::function<void(double)> consumer);

  /** Processes a value from a Spark only if the value is valid. */
  static void ifOk(rev::CANSparkMax &spark,
                   std::vector<std::function<double()>> suppliers,
                   std::function<void(std::vector<double>)> consumer);

  /** Return a value from a Spark (or the default if the value is invalid). */
  static double ifOkOrDefault(rev::CANSparkMax &spark,
                              std::function<double()> supplier,
                              double defaultValue);

  /**
   * Return a processed set of values from a Spark (or the default if one of the
   * values is invalid).
   */
  static double
  ifOkOrDefault(rev::CANSparkMax &spark,
                std::vector<std::function<double()>> suppliers,
                std::function<double(std::vector<double>)> transformer,
                double defaultValue);

  /** Attempts to run the command until no error is produced. */
  static void tryUntilOk(rev::CANSparkMax &spark, int maxAttempts,
                         std::function<rev::REVLibError()> command);
};

} // namespace frc2025::util