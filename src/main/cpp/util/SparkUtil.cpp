// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "SparkUtil.h"

namespace frc2025::util {

bool SparkUtil::sparkStickyFault = false;

void SparkUtil::ifOk(rev::CANSparkMax &spark, std::function<double()> supplier,
                     std::function<void(double)> consumer) {
  double value = supplier();
  if (spark.GetLastError() == rev::REVLibError::kOk) {
    consumer(value);
  } else {
    sparkStickyFault = true;
  }
}

void SparkUtil::ifOk(rev::CANSparkMax &spark,
                     std::vector<std::function<double()>> suppliers,
                     std::function<void(std::vector<double>)> consumer) {
  std::vector<double> values(suppliers.size());
  for (size_t i = 0; i < suppliers.size(); i++) {
    values[i] = suppliers[i]();
    if (spark.GetLastError() != rev::REVLibError::kOk) {
      sparkStickyFault = true;
      return;
    }
  }
  consumer(values);
}

double SparkUtil::ifOkOrDefault(rev::CANSparkMax &spark,
                                std::function<double()> supplier,
                                double defaultValue) {
  double value = supplier();
  if (spark.GetLastError() == rev::REVLibError::kOk) {
    return value;
  } else {
    sparkStickyFault = true;
    return defaultValue;
  }
}

double
SparkUtil::ifOkOrDefault(rev::CANSparkMax &spark,
                         std::vector<std::function<double()>> suppliers,
                         std::function<double(std::vector<double>)> transformer,
                         double defaultValue) {
  std::vector<double> values(suppliers.size());
  for (size_t i = 0; i < suppliers.size(); i++) {
    values[i] = suppliers[i]();
    if (spark.GetLastError() != rev::REVLibError::kOk) {
      sparkStickyFault = true;
      return defaultValue;
    }
  }
  return transformer(values);
}

void SparkUtil::tryUntilOk(rev::CANSparkMax &spark, int maxAttempts,
                           std::function<rev::REVLibError()> command) {
  for (int i = 0; i < maxAttempts; i++) {
    rev::REVLibError error = command();
    if (error == rev::REVLibError::kOk) {
      break;
    } else {
      sparkStickyFault = true;
    }
  }
}

} // namespace frc2025::util