// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure.dispenser;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.ElevatorIOSim;

public class PivotIOSim implements PivotIO {
  private static final double reduction = 3.0;

  public static final double moi =
      (0.5)
          * (ElevatorIOSim.carriageMassKg)
          * Math.pow(SuperstructureConstants.pivotToTunnelFront, 2.0);
  private static final double cgRadius = Units.inchesToMeters(10.0);
  public static final DCMotor gearbox = DCMotor.getKrakenX60Foc(1).withReduction(reduction);
  public static final Matrix<N2, N2> A =
      MatBuilder.fill(
          Nat.N2(),
          Nat.N2(),
          0,
          1,
          0,
          -gearbox.KtNMPerAmp / (gearbox.KvRadPerSecPerVolt * gearbox.rOhms * moi));
  public static final Vector<N2> B = VecBuilder.fill(0, gearbox.KtNMPerAmp / moi);

  // State given by pivot angle position and velocity
  // Input given by torque current to motor
  private Vector<N2> simState;
  private double inputTorqueCurrent = 0.0;
  private double pivotAppliedVolts = 0.0;

  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private double feedforward = 0.0;
  private boolean closedLoop = false;

  public PivotIOSim() {
    simState = VecBuilder.fill(Dispenser.maxAngle.getRadians() - 0.1, 0.0);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (!closedLoop) {
      controller.reset();
      update(Constants.loopPeriodSecs);
    } else {
      // Run control at 1khz
      for (int i = 0; i < Constants.loopPeriodSecs / (1.0 / 1000.0); i++) {
        setInputTorqueCurrent(controller.calculate(simState.get(0)) + feedforward);
        update(1.0 / 1000.0);
      }
    }
    // Pivot
    inputs.data =
        new PivotIOData(
            true,
            true,
            Rotation2d.fromRadians(simState.get(0) - Dispenser.maxAngle.getRadians()),
            Rotation2d.fromRadians(simState.get(0)),
            simState.get(0),
            simState.get(1),
            pivotAppliedVolts,
            0.0,
            inputTorqueCurrent,
            0.0);
  }

  @Override
  public void runOpenLoop(double output) {
    closedLoop = false;
    setInputTorqueCurrent(output);
  }

  @Override
  public void runVolts(double volts) {
    closedLoop = false;
    setInputVoltage(volts);
  }

  @Override
  public void stop() {
    runOpenLoop(0.0);
  }

  @Override
  public void runPosition(Rotation2d position, double feedforward) {
    closedLoop = true;
    controller.setSetpoint(position.getRadians() + Dispenser.maxAngle.getRadians());
    this.feedforward = feedforward;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }

  private void setInputTorqueCurrent(double torqueCurrent) {
    inputTorqueCurrent = torqueCurrent;
    pivotAppliedVolts =
        gearbox.getVoltage(gearbox.getTorque(inputTorqueCurrent), simState.get(1, 0));
    pivotAppliedVolts = MathUtil.clamp(pivotAppliedVolts, -12.0, 12.0);
  }

  private void setInputVoltage(double voltage) {
    setInputTorqueCurrent(gearbox.getCurrent(simState.get(1, 0), voltage));
  }

  private void update(double dt) {
    inputTorqueCurrent = MathUtil.clamp(inputTorqueCurrent, -40.0, 40.0);
    Matrix<N2, N1> updatedState =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> u) -> {
              Matrix<N2, N1> xdot = A.times(x).plus(B.times(u));
              // Add gravity
              xdot.plus(
                  -SuperstructureConstants.G
                      * cgRadius
                      * Rotation2d.fromRadians(simState.get(0))
                          .minus(SuperstructureConstants.elevatorAngle)
                          .getCos()
                      / moi);
              return xdot;
            },
            simState,
            VecBuilder.fill(inputTorqueCurrent),
            dt);
    // Apply limits
    simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
    if (simState.get(0) <= Dispenser.minAngle.getRadians()) {
      simState.set(1, 0, 0.0);
      simState.set(0, 0, Dispenser.minAngle.getRadians());
    }
    if (simState.get(0) >= Dispenser.maxAngle.getRadians()) {
      simState.set(1, 0, 0.0);
      simState.set(0, 0, Dispenser.maxAngle.getRadians());
    }
  }
}
