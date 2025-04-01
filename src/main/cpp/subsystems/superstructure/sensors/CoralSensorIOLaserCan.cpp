// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "CoralSensorIOLaserCan.h"

#include <iostream>

CoralSensorIOLaserCan::CoralSensorIOLaserCan() : laserCan(55) {
  try {
    laserCan.setTimingBudget(
        grapplerobotics::LaserCanInterface::TimingBudget::TIMING_BUDGET_20MS);
    laserCan.setRangingMode(
        grapplerobotics::LaserCanInterface::RangingMode::SHORT);
    laserCan.setRegionOfInterest(
        grapplerobotics::LaserCanInterface::RegionOfInterest(8, 8, 4, 4));
  } catch (const grapplerobotics::ConfigurationFailedException &e) {
    std::cerr << "ConfigurationFailedException: " << e.what() << std::endl;
  }
}

void CoralSensorIOLaserCan::UpdateInputs(
    CoralSensorIOInputsAutoLogged &inputs) {
  auto measurement = laserCan.getMeasurement();
  bool valid = measurement.has_value() &&
               measurement.value().status ==
                   grapplerobotics::LaserCan::LASERCAN_STATUS_VALID_MEASUREMENT;
  inputs.data.distanceMeters =
      valid ? static_cast<double>(measurement.value().distance_mm) / 1000.0
            : 0.0;
  inputs.data.valid = valid;
}