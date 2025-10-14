// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.sensors;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;

public class CoralSensorIOLaserCan implements CoralSensorIO {
  private final LaserCan laserCan;

  public CoralSensorIOLaserCan() {
    laserCan = new LaserCan(55);

    try {
      laserCan.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS);
      laserCan.setRangingMode(LaserCanInterface.RangingMode.SHORT);
      laserCan.setRegionOfInterest(new LaserCanInterface.RegionOfInterest(8, 8, 4, 4));
    } catch (ConfigurationFailedException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void updateInputs(CoralSensorIOInputs inputs) {
    var measurement = laserCan.getMeasurement();
    boolean valid =
        measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    inputs.data =
        new CoralSensorIOData(valid ? ((double) measurement.distance_mm) / 1000.0 : 0, valid);
  }
}
