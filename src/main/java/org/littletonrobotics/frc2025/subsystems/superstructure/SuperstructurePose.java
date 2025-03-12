// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import static org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.Dispenser;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;

public record SuperstructurePose(DoubleSupplier elevatorHeight, Supplier<Rotation2d> pivotAngle) {
  private static final LoggedTunableNumber intakeHeightBaseline =
      new LoggedTunableNumber("Superstructure/Intake/ElevatorBaseline", 0.04);
  private static final LoggedTunableNumber intakeHeightRange =
      new LoggedTunableNumber("Superstructure/Intake/ElevatorRange", 0.008);
  private static final LoggedTunableNumber intakeHeightTimeFactor =
      new LoggedTunableNumber("Superstructure/Intake/ElevatorTimeFactor", 25.0);

  private static final Map<ReefLevel, LoggedTunableNumber> ejectDistance = new HashMap<>();
  private static final Map<ReefLevel, LoggedTunableNumber> ejectAngles = new HashMap<>();
  private static final Map<ReefLevel, LoggedTunableNumber> heightFudges = new HashMap<>();

  private static void addInitialValue(
      Map<ReefLevel, LoggedTunableNumber> map,
      ReefLevel reefLevel,
      double initialValue,
      String key) {
    map.put(
        reefLevel,
        new LoggedTunableNumber("Superstructure/ReefScore/" + key + "/" + reefLevel, initialValue));
  }

  static {
    // Coral eject distance
    addInitialValue(ejectDistance, ReefLevel.L1, Units.inchesToMeters(2.0), "EjectDistance");
    addInitialValue(ejectDistance, ReefLevel.L2, Units.inchesToMeters(8.0), "EjectDistance");
    addInitialValue(ejectDistance, ReefLevel.L3, Units.inchesToMeters(5.5), "EjectDistance");
    addInitialValue(ejectDistance, ReefLevel.L4, Units.inchesToMeters(6.0), "EjectDistance");
    // Coral eject angles
    addInitialValue(ejectAngles, ReefLevel.L1, 20.0, "EjectAngles");
    addInitialValue(ejectAngles, ReefLevel.L2, -20.0, "EjectAngles");
    addInitialValue(ejectAngles, ReefLevel.L3, -35.0, "EjectAngles");
    addInitialValue(ejectAngles, ReefLevel.L4, -48.0, "EjectAngles");
    // Height fudges
    addInitialValue(heightFudges, ReefLevel.L1, 0.08, "HeightFudges");
    addInitialValue(heightFudges, ReefLevel.L2, Units.inchesToMeters(0.0), "HeightFudges");
    addInitialValue(heightFudges, ReefLevel.L3, Units.inchesToMeters(2.5), "HeightFudges");
    addInitialValue(heightFudges, ReefLevel.L4, Units.inchesToMeters(1.0), "HeightFudges");
  }

  @Getter
  @RequiredArgsConstructor
  /**
   * Dispenser pose relative to branch on ground (looking at the robot from the left side) Rotation
   * is just the rotation of dispenser when scoring.
   */
  public enum DispenserPose {
    L1(() -> getCoralScorePose(ReefLevel.L1)),
    L2(() -> getCoralScorePose(ReefLevel.L2)),
    L3(() -> getCoralScorePose(ReefLevel.L3)),
    L4(() -> getCoralScorePose(ReefLevel.L4));

    private final Supplier<Pose2d> pose;

    public double getElevatorHeight() {
      return (pose.get().getY() - dispenserOrigin2d.getY()) / elevatorAngle.getSin();
    }

    public double getDispenserAngleDeg() {
      return pose.get().getRotation().getDegrees();
    }

    public Transform2d toRobotPose() {
      return new Transform2d(
          getElevatorHeight() * SuperstructureConstants.elevatorAngle.getCos()
              + pose.get().getX()
              + SuperstructureConstants.dispenserOrigin2d.getX(),
          0.0,
          Rotation2d.kPi);
    }

    public static DispenserPose forCoralScore(ReefLevel reefLevel) {
      return switch (reefLevel) {
        case L1 -> DispenserPose.L1;
        case L2 -> DispenserPose.L2;
        case L3 -> DispenserPose.L3;
        case L4 -> DispenserPose.L4;
      };
    }

    private static Pose2d getCoralScorePose(ReefLevel reefLevel) {
      double dispenserAngleDeg = ejectAngles.get(reefLevel).get();
      if (Constants.getRobot() == Constants.RobotType.DEVBOT) {
        dispenserAngleDeg = -30.0;
      }
      return new Pose2d(
          new Pose2d(
                  0.0,
                  reefLevel.height + heightFudges.get(reefLevel).get(),
                  Rotation2d.fromDegrees(-dispenserAngleDeg))
              .transformBy(
                  GeomUtil.toTransform2d(
                      ejectDistance.get(reefLevel).get() + pivotToTunnelFront, 0.0))
              .getTranslation(),
          Rotation2d.fromDegrees(dispenserAngleDeg));
    }
  }

  // Read distance to branch from robot state to calculate positions
  @RequiredArgsConstructor
  @Getter
  enum Preset {
    STOW("Stow", intakeHeightBaseline.get(), -18.0),
    INTAKE(
        () ->
            intakeHeightBaseline.get()
                + intakeHeightRange.get()
                    * Math.sin(Timer.getTimestamp() * intakeHeightTimeFactor.get()),
        new LoggedTunableNumber("Superstructure/Intake/IntakePivot", -18.0)),
    GOODBYE_CORAL(
        intakeHeightBaseline, new LoggedTunableNumber("Superstructure/Intake/EjectPivot", 18.0)),
    L1(ReefLevel.L1, false),
    L1_EJECT(ReefLevel.L1, true),
    L2(ReefLevel.L2, false),
    L3(ReefLevel.L3, false),
    L4(ReefLevel.L4, false),
    ALGAE_L2_INTAKE("AlgaeIntakeL2", 1.5, 0.0),
    ALGAE_L3_INTAKE("AlgaeIntakeL3", 1.5, 0.0),
    PRE_THROW("PreThrow", 1.0, Dispenser.maxAngle.getDegrees()),
    THROW("Throw", elevatorMaxTravel, Dispenser.maxAngle.getDegrees()),
    ALGAE_STOW("AlgaeStow", 0.15, 0.0);

    private final SuperstructurePose pose;

    Preset(DoubleSupplier elevatorHeight, DoubleSupplier pivotAngle) {
      this(
          new SuperstructurePose(
              elevatorHeight, () -> Rotation2d.fromDegrees(pivotAngle.getAsDouble())));
    }

    Preset(String name, double elevatorHeight, double pivotAngle) {
      this(
          new LoggedTunableNumber("Superstructure/" + name + "/Elevator", elevatorHeight),
          new LoggedTunableNumber("Superstructure/" + name + "/Pivot", pivotAngle));
    }

    Preset(ReefLevel reefLevel, boolean isL1Eject) {
      var dispenserPose = DispenserPose.forCoralScore(reefLevel);
      DoubleSupplier l1LaunchAdjustment =
          isL1Eject
              ? new LoggedTunableNumber("Superstructure/ReefScore/L1LaunchAdjustment", 0.0)
              : () -> 0.0;
      pose =
          new SuperstructurePose(
              () -> dispenserPose.getElevatorHeight() + l1LaunchAdjustment.getAsDouble(),
              () ->
                  (RobotState.getInstance().getDistanceToBranch().isEmpty()
                          || reefLevel == ReefLevel.L1)
                      ? Rotation2d.fromDegrees(dispenserPose.getDispenserAngleDeg())
                      : Rotation2d.fromDegrees(
                          new Rotation2d(
                                  RobotState.getInstance().getDistanceToBranch().getAsDouble(),
                                  reefLevel.height
                                      - (dispenserPose.getElevatorHeight() * elevatorAngle.getSin()
                                          + dispenserOrigin2d.getY()))
                              .getDegrees()));
    }
  }

  SuperstructurePose() {
    this(() -> 0.0, () -> Rotation2d.kZero);
  }
}
