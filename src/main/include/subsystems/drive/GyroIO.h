// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#ifndef GYRO_IO_H
#define GYRO_IO_H

#include <cmath>
#include <vector>

namespace org {
namespace littletonrobotics {
namespace frc2025 {
namespace subsystems {
namespace drive {

class Rotation2d {
public:
  double cosValue;
  double sinValue;

  Rotation2d(double cosValue, double sinValue)
      : cosValue(cosValue), sinValue(sinValue) {}

  static Rotation2d fromRadians(double radians) {
    return Rotation2d(std::cos(radians), std::sin(radians));
  }

  double getRadians() const { return std::atan2(sinValue, cosValue); }

  static const Rotation2d kZero;
};

const Rotation2d Rotation2d::kZero = Rotation2d(1.0, 0.0);

class GyroIO {
public:
  struct GyroIOData {
    bool connected;
    Rotation2d yawPosition;
    double yawVelocityRadPerSec;

    GyroIOData(bool connected = false,
               Rotation2d yawPosition = Rotation2d::kZero,
               double yawVelocityRadPerSec = 0.0)
        : connected(connected), yawPosition(yawPosition),
          yawVelocityRadPerSec(yawVelocityRadPerSec) {}
  };

  struct GyroIOInputs {
    GyroIOData data;
    std::vector<double> odometryYawTimestamps;
    std::vector<Rotation2d> odometryYawPositions;

    GyroIOInputs() : data(GyroIOData()) {}
  };

  virtual ~GyroIO() {}
  virtual void updateInputs(GyroIOInputs &inputs) {}
};

} // namespace drive
} // namespace subsystems
} // namespace frc2025
} // namespace littletonrobotics
} // namespace org

#endif // GYRO_IO_H