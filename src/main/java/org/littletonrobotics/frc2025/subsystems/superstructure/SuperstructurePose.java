// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import static org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
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
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;

public record SuperstructurePose(DoubleSupplier elevatorHeight, Supplier<Rotation2d> pivotAngle) {
  private static final LoggedTunableNumber intakeHeightBaseline =
      new LoggedTunableNumber("Superstructure/Intake/ElevatorBaseline", 0.0);
  private static final LoggedTunableNumber intakeHeightRange =
      new LoggedTunableNumber("Superstructure/Intake/ElevatorRange", 0.0);
  private static final LoggedTunableNumber intakeHeightTimeFactor =
      new LoggedTunableNumber("Superstructure/Intake/ElevatorTimeFactor", 25.0);
  private static final LoggedTunableNumber l1Height =
      new LoggedTunableNumber("Superstructure/L1/Elevator", 0.48);
  private static final LoggedTunableNumber l1Pivot =
      new LoggedTunableNumber("Superstructure/L1/Pivot", 0.0);
  private static final LoggedTunableNumber l1LaunchAdjustment =
      new LoggedTunableNumber("Superstructure/L1/LaunchAdjustment", 0.0);
  public static final LoggedTunableNumber algaeSuperPositionDeg =
      new LoggedTunableNumber("Superstructure/ReefScore/AlgaeSuperPositionDegrees", -10.0);
  private static final LoggedTunableNumber l2ReefIntakeHeight =
      new LoggedTunableNumber("Superstructure/AlgaeIntake/L2/Elevator", 0.65);
  private static final LoggedTunableNumber l3ReefIntakeHeight =
      new LoggedTunableNumber("Superstructure/AlgaeIntake/L3/Elevator", 1.0);
  private static final LoggedTunableNumber reefIntakeHeightAdjustment =
      new LoggedTunableNumber("Superstructure/AlgaeIntake/HeightAdjustment", 0.05);
  private static final LoggedTunableNumber reefIntakeMinDispenserDistance =
      new LoggedTunableNumber("Superstructure/AlgaeIntake/MinDispenserDistance", 0.18);
  private static final LoggedTunableNumber reefIntakeMaxDispenserDistance =
      new LoggedTunableNumber("Superstructure/AlgaeIntake/MaxDispenserDistance", 0.4);

  private static final Map<ReefLevel, Pair<LoggedTunableNumber, LoggedTunableNumber>>
      ejectDistance = new HashMap<>();
  private static final Map<ReefLevel, Pair<LoggedTunableNumber, LoggedTunableNumber>> ejectAngles =
      new HashMap<>();
  private static final Map<ReefLevel, Pair<LoggedTunableNumber, LoggedTunableNumber>> heightFudges =
      new HashMap<>();

  private static void addInitialValue(
      Map<ReefLevel, Pair<LoggedTunableNumber, LoggedTunableNumber>> map,
      ReefLevel reefLevel,
      double initialValue,
      double initialValueAlgae,
      String key) {
    map.put(
        reefLevel,
        Pair.of(
            new LoggedTunableNumber(
                "Superstructure/ReefScore/" + key + "/" + reefLevel, initialValue),
            new LoggedTunableNumber(
                "Superstructure/ReefScore/" + key + "Algae/" + reefLevel, initialValueAlgae)));
  }

  static {
    // Coral eject distance
    addInitialValue(ejectDistance, ReefLevel.L2, 0.22, 0.24, "EjectDistance");
    addInitialValue(ejectDistance, ReefLevel.L3, 0.22, 0.24, "EjectDistance");
    addInitialValue(ejectDistance, ReefLevel.L4, 0.12, 0.12, "EjectDistance");
    // Coral eject angles
    addInitialValue(ejectAngles, ReefLevel.L2, -20.0, 0.0, "EjectAngles");
    addInitialValue(ejectAngles, ReefLevel.L3, -20.0, 0.0, "EjectAngles");
    addInitialValue(ejectAngles, ReefLevel.L4, -48.0, -48.0, "EjectAngles");
    // Height fudges
    addInitialValue(
        heightFudges,
        ReefLevel.L2,
        Units.inchesToMeters(2.5),
        Units.inchesToMeters(2.5),
        "HeightFudges");
    addInitialValue(
        heightFudges,
        ReefLevel.L3,
        Units.inchesToMeters(2.5),
        Units.inchesToMeters(2.5),
        "HeightFudges");
    addInitialValue(
        heightFudges,
        ReefLevel.L4,
        Units.inchesToMeters(1.0),
        Units.inchesToMeters(1.0),
        "HeightFudges");
  }

  @Getter
  @RequiredArgsConstructor
  public enum CoralDispenserPose {
    L2(ReefLevel.L2, false),
    L3(ReefLevel.L3, false),
    L4(ReefLevel.L4, false),
    ALGAE_L2(ReefLevel.L2, true),
    ALGAE_L3(ReefLevel.L3, true),
    ALGAE_L4(ReefLevel.L4, true);

    private final Supplier<Pose2d> pose;
    private final ReefLevel reefLevel;
    private final boolean algae;

    CoralDispenserPose(ReefLevel reefLevel, boolean algae) {
      pose = () -> calculatePose(reefLevel, algae);
      this.reefLevel = reefLevel;
      this.algae = algae;
    }

    public double getElevatorHeight() {
      return (pose.get().getY() - dispenserOrigin2d.getY()) / elevatorAngle.getSin();
    }

    public double getDispenserAngleDeg() {
      return pose.get().getRotation().getDegrees();
    }

    public Transform2d branchToRobot() {
      return new Transform2d(
          getElevatorHeight() * elevatorAngle.getCos()
              + pose.get().getX()
              + dispenserOrigin2d.getX(),
          0.0,
          Rotation2d.kPi);
    }

    public Transform2d robotToDispenser() {
      return GeomUtil.toTransform2d(
          getElevatorHeight() * SuperstructureConstants.elevatorAngle.getCos()
              + SuperstructureConstants.dispenserOrigin2d.getX(),
          0.0);
    }

    private static Pose2d calculatePose(ReefLevel reefLevel, boolean algae) {
      double dispenserAngleDeg =
          algae
              ? ejectAngles.get(reefLevel).getSecond().get()
              : ejectAngles.get(reefLevel).getFirst().get();
      if (Constants.getRobot() == Constants.RobotType.DEVBOT) {
        dispenserAngleDeg = -30.0;
      }
      return new Pose2d(
          new Pose2d(
                  0.0,
                  reefLevel.height
                      + (Constants.getRobot() == Constants.RobotType.SIMBOT
                          ? 0.0
                          : (algae
                              ? heightFudges.get(reefLevel).getSecond().get()
                              : heightFudges.get(reefLevel).getFirst().get())),
                  Rotation2d.fromDegrees(-dispenserAngleDeg))
              .transformBy(
                  GeomUtil.toTransform2d(
                      (algae
                              ? ejectDistance.get(reefLevel).getSecond().get()
                              : ejectDistance.get(reefLevel).getFirst().get())
                          + pivotToTunnelFront,
                      0.0))
              .getTranslation(),
          Rotation2d.fromDegrees(dispenserAngleDeg));
    }
  }

  // Read distance to branch from robot state to calculate positions
  @RequiredArgsConstructor
  @Getter
  enum Preset {
    STOW(intakeHeightBaseline, () -> -18.0),
    CORAL_INTAKE(
        () ->
            intakeHeightBaseline.get()
                + intakeHeightRange.get()
                    * Math.sin(Timer.getTimestamp() * intakeHeightTimeFactor.get()),
        new LoggedTunableNumber("Superstructure/Intake/IntakePivot", -18.0)),
    GOODBYE_CORAL(
        intakeHeightBaseline, new LoggedTunableNumber("Superstructure/Intake/EjectPivot", 0.0)),
    L1(l1Height, l1Pivot),
    L1_EJECT(() -> l1Height.get() + l1LaunchAdjustment.get(), l1Pivot),
    L2(CoralDispenserPose.L2),
    L3(CoralDispenserPose.L3),
    L4(CoralDispenserPose.L4),
    ALGAE_L2(CoralDispenserPose.ALGAE_L2),
    ALGAE_L3(CoralDispenserPose.ALGAE_L3),
    ALGAE_L4(CoralDispenserPose.ALGAE_L4),
    ALGAE_FLOOR_INTAKE("AlgaeFloorIntake", 0.4, -20.0),
    ALGAE_L2_INTAKE(
        () -> l2ReefIntakeHeight.get() + getReefIntakeAdjustment(),
        new LoggedTunableNumber("Superstructure/AlgaeIntake/L2/Pivot", -55.0)),
    ALGAE_L3_INTAKE(
        () -> l3ReefIntakeHeight.get() + getReefIntakeAdjustment(),
        new LoggedTunableNumber("Superstructure/AlgaeIntake/L3/Pivot", -55.0)),
    ALGAE_ICE_CREAM_INTAKE("AlgaeIceCreamIntake", 0.15, -45.0),
    THROW("Throw", elevatorMaxTravel, 0.0),
    ALGAE_STOW("AlgaeStow", intakeHeightBaseline.get(), -15.0),
    PROCESS("Processed", 0.21, -70.0);

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

    Preset(CoralDispenserPose coralDispenserPose) {
      pose =
          new SuperstructurePose(
              coralDispenserPose::getElevatorHeight,
              () ->
                  RobotState.getInstance().getDistanceToBranch().isEmpty()
                      ? Rotation2d.fromDegrees(coralDispenserPose.getDispenserAngleDeg())
                      : Rotation2d.fromDegrees(
                          new Rotation2d(
                                  RobotState.getInstance().getDistanceToBranch().getAsDouble(),
                                  coralDispenserPose.getReefLevel().height
                                      - (coralDispenserPose.getElevatorHeight()
                                              * elevatorAngle.getSin()
                                          + dispenserOrigin2d.getY()))
                              .getDegrees()));
    }

    private static double getReefIntakeAdjustment() {
      var distanceToAlgae = RobotState.getInstance().getDistanceToReefAlgae();
      if (distanceToAlgae.isEmpty()) return 0.0;
      return MathUtil.clamp(
              (distanceToAlgae.getAsDouble() - reefIntakeMinDispenserDistance.get())
                  / (reefIntakeMaxDispenserDistance.get() - reefIntakeMinDispenserDistance.get()),
              0.0,
              1.0)
          * reefIntakeHeightAdjustment.get();
    }
  }
}
