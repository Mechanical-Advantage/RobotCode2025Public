// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.util.gslam;

import edu.wpi.first.math.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.frc2025.Constants;

public class GenericSlamIOSim implements GenericSlamIO {
  private final DCMotor gearbox;
  private final double maxTravel;
  private final Matrix<N2, N2> A;
  private final Vector<N2> B;

  protected Vector<N2> simState = VecBuilder.fill(0.0, 0.0);
  private double inputTorqueCurrent = 0.0;
  private double appliedVolts = 0.0;

  public GenericSlamIOSim(DCMotor gearbox, double moi, double maxTravel) {
    this.gearbox = gearbox;
    this.maxTravel = maxTravel;
    A =
        MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            0,
            1,
            0,
            -gearbox.KtNMPerAmp / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * moi));
    B = VecBuilder.fill(0.0, gearbox.KtNMPerAmp / moi);
  }

  @Override
  public void updateInputs(GenericSlamIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    update(Constants.loopPeriodSecs);
    inputs.data =
        new GenericSlamIOData(
            true,
            simState.get(0),
            simState.get(1),
            appliedVolts,
            (appliedVolts / 12.0) * inputTorqueCurrent,
            inputTorqueCurrent,
            0.0);
  }

  @Override
  public void runTorqueCurrent(double amps) {
    inputTorqueCurrent = amps;
    appliedVolts = gearbox.getVoltage(gearbox.getTorque(inputTorqueCurrent), simState.get(1, 0));
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
  }

  @Override
  public void stop() {
    runTorqueCurrent(0.0);
  }

  private void update(double dt) {
    inputTorqueCurrent = MathUtil.clamp(inputTorqueCurrent, -40.0, 40.0);
    Matrix<N2, N1> updatedState =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> u) -> A.times(x).plus(B.times(u)),
            simState,
            VecBuilder.fill(inputTorqueCurrent * 15.0),
            dt);
    // Apply limits
    simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    if (simState.get(0) <= 0.0) {
      simState.set(1, 0, 0.0);
      simState.set(0, 0, 0.0);
    }
    if (simState.get(0) >= maxTravel) {
      simState.set(1, 0, 0.0);
      simState.set(0, 0, maxTravel);
    }
  }
}
