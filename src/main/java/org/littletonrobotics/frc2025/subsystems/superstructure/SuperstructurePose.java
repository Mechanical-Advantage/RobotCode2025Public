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
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;

public record SuperstructurePose(DoubleSupplier elevatorHeight, Supplier<Rotation2d> pivotAngle) {
  private static final double reefAlgaeIntakeAngleL2 = -20.0;
  private static final double reefAlgaeIntakeAngleL3 = 0.0;
  private static final double reefAlgaeIntakeDispenserAngleL2 = 5.0;
  private static final double reefAlgaeIntakeDispenserAngleL3 = -20.0;

  private static final LoggedTunableNumber intakeHeightBaseline =
      new LoggedTunableNumber("Superstructure/Intake/ElevatorBaseline", 0.04);
  private static final LoggedTunableNumber intakeHeightRange =
      new LoggedTunableNumber("Superstructure/Intake/ElevatorRange", 0.008);
  private static final LoggedTunableNumber intakeHeightTimeFactor =
      new LoggedTunableNumber("Superstructure/Intake/ElevatorTimeFactor", 25.0);

  private static final Map<ReefLevel, Double> ejectMeters =
      Map.of(
          ReefLevel.L1,
          Units.inchesToMeters(2.0),
          ReefLevel.L2,
          Units.inchesToMeters(5.0),
          ReefLevel.L3,
          Units.inchesToMeters(5.0),
          ReefLevel.L4,
          Units.inchesToMeters(5.0));
  private static final Map<ReefLevel, Double> heightFudges =
      Map.of(
          ReefLevel.L1,
          Units.inchesToMeters(2.0),
          ReefLevel.L2,
          Units.inchesToMeters(3.0),
          ReefLevel.L3,
          Units.inchesToMeters(3.0),
          ReefLevel.L4,
          Units.inchesToMeters(1.0));
  private static final Map<ReefLevel, Double> optimalCoralAngles =
      Map.of(ReefLevel.L1, 0.0, ReefLevel.L2, -35.0, ReefLevel.L3, -35.0, ReefLevel.L4, -48.0);

  @Getter
  @RequiredArgsConstructor
  /**
   * Dispenser pose relative to branch on ground (looking at the robot from the left side) Rotation
   * is just the rotation of dispenser when scoring.
   */
  public enum DispenserPose {
    L1(getCoralScorePose(ReefLevel.L1)),
    L2(getCoralScorePose(ReefLevel.L2)),
    L3(getCoralScorePose(ReefLevel.L3)),
    L4(getCoralScorePose(ReefLevel.L4)),
    L2_ALGAE_INTAKE(getAlgaeIntakePose(true)),
    L3_ALGAE_INTAKE(getAlgaeIntakePose(false));

    private final Pose2d pose;

    public double getElevatorHeight() {
      return (pose.getY() - dispenserOrigin2d.getY()) / elevatorAngle.getSin();
    }

    public double getDispenserAngleDeg() {
      return pose.getRotation().getDegrees();
    }

    public Transform2d toRobotPose() {
      return new Transform2d(
          getElevatorHeight() * SuperstructureConstants.elevatorAngle.getCos()
              + pose.getX()
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

    public static DispenserPose forAlgaeIntake(FieldConstants.AlgaeObjective objective) {
      return (objective.id() % 2 == 0)
          ? DispenserPose.L3_ALGAE_INTAKE
          : DispenserPose.L2_ALGAE_INTAKE;
    }

    private static Pose2d getCoralScorePose(ReefLevel reefLevel) {
      double dispenserAngleDeg = optimalCoralAngles.get(reefLevel);
      if (Constants.getRobot() == Constants.RobotType.DEVBOT) {
        dispenserAngleDeg = -30.0;
      }
      return new Pose2d(
          new Pose2d(
                  0.0,
                  reefLevel.height + heightFudges.get(reefLevel),
                  Rotation2d.fromDegrees(-dispenserAngleDeg))
              .transformBy(
                  GeomUtil.toTransform2d(ejectMeters.get(reefLevel) + pivotToTunnelFront, 0.0))
              .getTranslation(),
          Rotation2d.fromDegrees(dispenserAngleDeg));
    }

    private static Pose2d getAlgaeIntakePose(boolean L2Algae) {
      return new Pose2d(
          new Pose2d(
                  new Pose2d(
                          0.0,
                          ReefLevel.L3.height
                              + (L2Algae ? -1.0 : 1.0)
                                  * (ReefLevel.L3.height - ReefLevel.L2.height)
                                  / 2.0,
                          Rotation2d.fromDegrees(
                              -180.0 - ReefLevel.L3.pitch)) // Flip pitch into frame
                      .transformBy(GeomUtil.toTransform2d(FieldConstants.algaeDiameter / 2.0, 0.0))
                      .getTranslation(),
                  Rotation2d.fromDegrees(
                      -(L2Algae ? reefAlgaeIntakeAngleL2 : reefAlgaeIntakeAngleL3)))
              .transformBy(
                  GeomUtil.toTransform2d(
                      FieldConstants.algaeDiameter / 2.0
                          + pivotToTunnelFront
                          + Units.inchesToMeters(3.0),
                      0.0))
              .getTranslation(),
          Rotation2d.fromDegrees(
              L2Algae ? reefAlgaeIntakeDispenserAngleL2 : reefAlgaeIntakeDispenserAngleL3));
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
        new LoggedTunableNumber("Superstructure/Intake/Pivot", -18.0)),
    L1(ReefLevel.L1),
    L2(ReefLevel.L2),
    L3(ReefLevel.L3),
    L4(ReefLevel.L4),
    ALGAE_FLOOR_INTAKE("AlgaeFloorIntake", 0.4, -20.0),
    ALGAE_L2_INTAKE("AlgaeL2Intake", DispenserPose.L2_ALGAE_INTAKE),
    ALGAE_L3_INTAKE("AlgaeL3Intake", DispenserPose.L3_ALGAE_INTAKE),
    PRE_THROW("PreThrow", elevatorMaxTravel, 20.0),
    THROW("Throw", elevatorMaxTravel, 20.0),
    ALGAE_STOW("AlgaeStow", 0.15, 0.0);

    private final SuperstructurePose pose;

    Preset(DoubleSupplier elevatorHeight, DoubleSupplier pivotAngle) {
      this(
          new SuperstructurePose(
              elevatorHeight, () -> Rotation2d.fromDegrees(pivotAngle.getAsDouble())));
    }

    Preset(String name, DispenserPose dispenserPose) {
      this(name, dispenserPose.getElevatorHeight(), dispenserPose.getDispenserAngleDeg());
    }

    Preset(String name, double elevatorHeight, double pivotAngle) {
      this(
          new LoggedTunableNumber("Superstructure/" + name + "/Elevator", elevatorHeight),
          new LoggedTunableNumber("Superstructure/" + name + "/Pivot", pivotAngle));
    }

    Preset(ReefLevel ReefLevel) {
      var dispenserPose = DispenserPose.forCoralScore(ReefLevel);
      var elevatorHeight =
          new LoggedTunableNumber(
              "Superstructure/" + ReefLevel + "/Elevator", dispenserPose.getElevatorHeight());
      pose =
          new SuperstructurePose(
              elevatorHeight,
              () ->
                  RobotState.getInstance().getDistanceToBranch().isEmpty()
                      ? Rotation2d.fromDegrees(dispenserPose.getDispenserAngleDeg())
                      : Rotation2d.fromDegrees(
                          new Rotation2d(
                                  RobotState.getInstance().getDistanceToBranch().getAsDouble(),
                                  ReefLevel.height
                                      - (elevatorHeight.get() * elevatorAngle.getSin()
                                          + dispenserOrigin2d.getY()))
                              .getDegrees()));
    }
  }

  SuperstructurePose() {
    this(() -> 0.0, () -> Rotation2d.kZero);
  }
}
