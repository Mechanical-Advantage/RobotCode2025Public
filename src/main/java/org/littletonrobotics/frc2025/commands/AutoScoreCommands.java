// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.*;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.AlgaeObjective;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.FieldConstants.Reef;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructurePose.DispenserPose;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.Container;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AutoScoreCommands {
  // Radius of regular hexagon is side length
  private static final double reefRadius = Reef.faceLength;
  private static final LoggedTunableNumber maxDistanceReefLineup =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineup", 1.5);
  public static final LoggedTunableNumber minDistanceReefClearAlgae =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClearAlgae", Units.inchesToMeters(18.0));
  public static final LoggedTunableNumber algaeBackupTime =
      new LoggedTunableNumber("AutoScore/AlgaeBackupTime", 1.2);
  public static final LoggedTunableNumber minDistanceReefClear =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClear", Units.inchesToMeters(12.0));
  private static final LoggedTunableNumber distanceSuperstructureReady =
      new LoggedTunableNumber("AutoScore/DistanceSuperstructureReady", Units.inchesToMeters(72.0));
  private static final LoggedTunableNumber[] linearXToleranceEject = {
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L1", 0.03),
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L2", 0.15),
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L3", 0.15),
    new LoggedTunableNumber("AutoScore/LinearXToleranceEject/L4", 0.025)
  };
  private static final LoggedTunableNumber[] linearYToleranceEject = {
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L1", 0.03),
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L2", 0.015),
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L3", 0.015),
    new LoggedTunableNumber("AutoScore/LinearYToleranceEject/L4", 0.01)
  };
  private static final LoggedTunableNumber[] maxLinearVel = {
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L1", 0.1),
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L2", 0.1),
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L3", 0.1),
    new LoggedTunableNumber("AutoScore/MaxLinearVel/L4", 0.1)
  };
  private static final LoggedTunableNumber[] maxAngularVel = {
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L1", 1),
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L2", 1),
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L3", 1),
    new LoggedTunableNumber("AutoScore/MaxAngularVel/L4", 1)
  };
  private static final LoggedTunableNumber thetaToleranceEject =
      new LoggedTunableNumber("AutoScore/ThetaToleranceEject", 2.0);
  private static final LoggedTunableNumber l1AlignOffsetX =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetX", 0.45);
  private static final LoggedTunableNumber l1AlignOffsetY =
      new LoggedTunableNumber(
          "AutoScore/L1AlignOffsetY",
          FieldConstants.Reef.faceLength / 2.0 - Units.inchesToMeters(2.5));
  private static final LoggedTunableNumber l1AlignOffsetDegrees =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetDegrees", 180.0);
  private static final LoggedTunableNumber minDistanceAim =
      new LoggedTunableNumber("AutoScore/MinDistanceAim", 0.2);
  private static final LoggedTunableNumber ejectTimeSeconds =
      new LoggedTunableNumber("AutoScore/EjectTimeSeconds", 0.5);

  private AutoScoreCommands() {}

  public static Command autoScore(
      Drive drive,
      Superstructure superstructure,
      RollerSystem funnel,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      Trigger disableReefAutoAlign,
      BooleanSupplier manualEject) {
    Supplier<Pose2d> robot =
        () ->
            coralObjective
                .get()
                .map(AutoScoreCommands::getRobotPose)
                .orElseGet(() -> RobotState.getInstance().getEstimatedPose());

    Function<CoralObjective, Pose2d> goal =
        objective ->
            objective.reefLevel() == ReefLevel.L1
                ? getL1Pose(objective)
                : getCoralScorePose(objective);

    Container<CoralObjective> coralObjectiveScored = new Container<>();
    Container<Boolean> needsToGetBack = new Container<>(false);
    Container<Boolean> hasEnded = new Container<>(false);

    var driveCommand =
        new DriveToPose(
            drive,
            () ->
                coralObjective
                    .get()
                    .map(
                        objective -> {
                          if (reefLevel.get() == ReefLevel.L1) {
                            return getDriveTarget(
                                robot.get(), AllianceFlipUtil.apply(getL1Pose(objective)));
                          }
                          Pose2d goalPose = getCoralScorePose(objective);
                          if (superstructure.getState()
                                  != Superstructure.getScoringState(reefLevel.get(), false)
                              && superstructure.getState()
                                  != Superstructure.getScoringState(reefLevel.get(), true)) {
                            goalPose =
                                goalPose.transformBy(
                                    GeomUtil.toTransform2d(
                                        goalPose.getTranslation().getDistance(Reef.center)
                                            - reefRadius
                                            - (DriveConstants.robotWidth / 2.0)
                                            - minDistanceReefClear.get(),
                                        0.0));
                          }
                          return getDriveTarget(robot.get(), AllianceFlipUtil.apply(goalPose));
                        })
                    .orElseGet(() -> RobotState.getInstance().getEstimatedPose()),
            robot,
            () ->
                DriveCommands.getLinearVelocityFromJoysticks(
                        driverX.getAsDouble(), driverY.getAsDouble())
                    .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
            () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble()));

    // Schedule get back command
    new Trigger(() -> hasEnded.value && needsToGetBack.value)
        .and(RobotModeTriggers.teleop())
        .and(disableReefAutoAlign.negate())
        .onTrue(
            superstructureGetBack(superstructure, superstructure::getState, disableReefAutoAlign)
                .andThen(() -> needsToGetBack.value = false)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    return Commands.runOnce(
            () -> {
              // Start LEDs
              Leds.getInstance().autoScoringReef = true;
              Leds.getInstance().autoScoringLevel = reefLevel.get();

              // Reset state
              needsToGetBack.value = false;
              hasEnded.value = false;

              // Log reef level
              Logger.recordOutput("AutoScore/ReefLevel", reefLevel.get().toString());

              // Clear logs
              Logger.recordOutput("AutoScore/AllowPreReady", false);
              Logger.recordOutput("AutoScore/AllowEject", false);
            })
        .andThen(
            // Run superstructure
            IntakeCommands.intake(superstructure, funnel).until(superstructure::hasCoral),
            // Check if need wait until pre ready or already ready
            Commands.waitUntil(
                () -> {
                  boolean ready =
                      withinDistanceToReef(robot.get(), distanceSuperstructureReady.get())
                          || disableReefAutoAlign.getAsBoolean();
                  Logger.recordOutput("AutoScore/AllowPreReady", ready);

                  // Get back!
                  if (ready) {
                    needsToGetBack.value = true;
                  }
                  return ready;
                }),
            superstructureAimAndEject(
                superstructure,
                reefLevel,
                coralObjective,
                () -> {
                  if (coralObjective.get().isEmpty()) return false;
                  Pose2d poseError =
                      AllianceFlipUtil.apply(robot.get())
                          .relativeTo(goal.apply(coralObjective.get().get()));

                  int intReefLevel = coralObjective.get().get().reefLevel().ordinal();
                  boolean ready =
                      (Math.abs(poseError.getTranslation().getX())
                                  <= linearXToleranceEject[intReefLevel].get()
                              && Math.abs(poseError.getTranslation().getY())
                                  <= linearYToleranceEject[intReefLevel].get()
                              && Math.hypot(
                                      RobotState.getInstance().getRobotVelocity().vxMetersPerSecond,
                                      RobotState.getInstance().getRobotVelocity().vyMetersPerSecond)
                                  <= maxLinearVel[intReefLevel].get()
                              && Math.abs(
                                      RobotState.getInstance()
                                          .getRobotVelocity()
                                          .omegaRadiansPerSecond)
                                  <= maxAngularVel[intReefLevel].get()
                              && Math.abs(poseError.getRotation().getDegrees())
                                  <= thetaToleranceEject.get()
                              && superstructure.atGoal()
                              && !disableReefAutoAlign.getAsBoolean())
                          || manualEject.getAsBoolean();
                  Logger.recordOutput("AutoScore/AllowEject", ready);
                  if (ready) {
                    coralObjectiveScored.value = coralObjective.get().get();
                  }
                  return ready;
                },
                disableReefAutoAlign))
        .deadlineFor(
            Commands.either(
                joystickDrive, driveCommand, disableReefAutoAlign)) // Deadline with driving command
        .finallyDo(
            interrupted -> {
              RobotState.getInstance().setDistanceToBranch(OptionalDouble.empty());

              // Clear logs
              Logger.recordOutput("AutoScore/ReefLevel", "");
              Logger.recordOutput("AutoScore/AllowPreReady", false);
              Logger.recordOutput("AutoScore/AllowEject", false);

              // Stop LEDs
              Leds.getInstance().autoScoringReef = false;

              // Indicate has ended command
              hasEnded.value = true;
            });
  }

  public static Command autoScore(
      Drive drive,
      Superstructure superstructure,
      RollerSystem funnel,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective) {
    return autoScore(
        drive,
        superstructure,
        funnel,
        reefLevel,
        coralObjective,
        () -> 0,
        () -> 0,
        () -> 0,
        Commands.none(),
        new Trigger(() -> false),
        () -> false);
  }

  public static Command reefIntake(
      Drive drive,
      Superstructure superstructure,
      Supplier<Optional<AlgaeObjective>> algaeObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      Trigger disableReefAutoAlign) {
    Supplier<Pose2d> robot =
        () ->
            algaeObjective
                .get()
                .map(AutoScoreCommands::getRobotPose)
                .orElseGet(() -> RobotState.getInstance().getEstimatedPose());

    Supplier<SuperstructureState> algaeIntakeState =
        () ->
            algaeObjective
                .get()
                .map(
                    objective ->
                        objective.id() % 2 == 0
                            ? SuperstructureState.ALGAE_L3_INTAKE
                            : SuperstructureState.ALGAE_L2_INTAKE)
                .orElseGet(superstructure::getState);

    Container<AlgaeObjective> algaeIntaked = new Container<>();
    Container<Boolean> needsToGetBack = new Container<>(false);
    Container<Boolean> hasEnded = new Container<>(false);

    Timer hasAlgaeTimer = new Timer();
    hasAlgaeTimer.start();
    Supplier<Pose2d> goal =
        () -> {
          Pose2d goalPose =
              algaeObjective
                  .get()
                  .map(objective -> AllianceFlipUtil.apply(getReefIntakePose(objective)))
                  .orElseGet(
                      () ->
                          algaeIntaked.value == null
                              ? robot.get()
                              : AllianceFlipUtil.apply(getReefIntakePose(algaeIntaked.value)));
          if (algaeObjective.get().isEmpty() && algaeIntaked.value == null) {
            return goalPose;
          }
          if (superstructure.hasAlgae()) {
            return goalPose.transformBy(
                GeomUtil.toTransform2d(
                    -minDistanceReefClearAlgae.get()
                        * Math.min(1.0, hasAlgaeTimer.get() / algaeBackupTime.get()),
                    0.0));
          } else {
            hasAlgaeTimer.restart();
          }
          return goalPose;
        };

    // Schedule get back command
    new Trigger(() -> hasEnded.value && needsToGetBack.value)
        .and(RobotModeTriggers.teleop())
        .and(disableReefAutoAlign.negate())
        .onTrue(
            superstructureGetBack(superstructure, algaeIntakeState, disableReefAutoAlign)
                .andThen(() -> needsToGetBack.value = false)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    return Commands.runOnce(
            () -> {
              // Reset State
              algaeIntaked.value = null;
              needsToGetBack.value = false;
              hasEnded.value = false;
            })
        .andThen(
            Commands.either(
                    joystickDrive,
                    new DriveToPose(
                        drive,
                        () -> getDriveTarget(robot.get(), goal.get()),
                        robot,
                        () ->
                            DriveCommands.getLinearVelocityFromJoysticks(
                                    driverX.getAsDouble(), driverY.getAsDouble())
                                .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
                        () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble())),
                    disableReefAutoAlign)
                .alongWith(
                    superstructure
                        .runGoal(algaeIntakeState)
                        .alongWith(
                            Commands.waitUntil(
                                () -> superstructure.getState() == algaeIntakeState.get()),
                            Commands.runOnce(() -> needsToGetBack.value = true)),
                    Commands.waitUntil(superstructure::hasAlgae)
                        .andThen(
                            Commands.runOnce(
                                    () -> {
                                      algaeIntaked.value = algaeObjective.get().orElse(null);
                                    })
                                .onlyIf(() -> algaeObjective.get().isPresent()))))
        .finallyDo(() -> hasEnded.value = true);
  }

  public static Command superstructureAimAndEject(
      Superstructure superstructure,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective,
      BooleanSupplier eject,
      BooleanSupplier disableReefAutoAlign) {
    final Timer ejectTimer = new Timer();
    return superstructure
        .runGoal(() -> Superstructure.getScoringState(reefLevel.get(), false))
        .until(eject)
        .andThen(
            Commands.runOnce(ejectTimer::restart),
            superstructure
                .runGoal(() -> Superstructure.getScoringState(reefLevel.get(), true))
                .until(() -> ejectTimer.hasElapsed(ejectTimeSeconds.get())))
        .deadlineFor(
            // Measure distance to branch
            Commands.run(
                () -> {
                  coralObjective
                      .get()
                      .ifPresentOrElse(
                          objective -> {
                            if (disableReefAutoAlign.getAsBoolean()) {
                              RobotState.getInstance().setDistanceToBranch(OptionalDouble.empty());
                              return;
                            }

                            if (objective.reefLevel() == ReefLevel.L1) {
                              RobotState.getInstance().setDistanceToBranch(OptionalDouble.empty());
                              return;
                            }

                            var dispenserPose =
                                AllianceFlipUtil.apply(
                                    getRobotPose(objective)
                                        .transformBy(
                                            GeomUtil.toTransform2d(
                                                DispenserPose.forCoralScore(objective.reefLevel())
                                                            .getElevatorHeight()
                                                        * SuperstructureConstants.elevatorAngle
                                                            .getCos()
                                                    + SuperstructureConstants.dispenserOrigin2d
                                                        .getX(),
                                                0.0)));
                            var offsetTranslation =
                                dispenserPose
                                    .relativeTo(
                                        getBranchPose(objective)
                                            .transformBy(GeomUtil.toTransform2d(Rotation2d.kPi)))
                                    .getTranslation();
                            double distanceToBranch = offsetTranslation.getNorm();
                            Logger.recordOutput("AutoScore/DistanceToBranch", distanceToBranch);
                            RobotState.getInstance()
                                .setDistanceToBranch(
                                    distanceToBranch <= minDistanceAim.get()
                                        ? OptionalDouble.empty()
                                        : OptionalDouble.of(distanceToBranch));
                          },
                          () ->
                              RobotState.getInstance().setDistanceToBranch(OptionalDouble.empty()));
                }));
  }

  public static Command superstructureAimAndEject(
      Superstructure superstructure,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective,
      BooleanSupplier eject) {
    return superstructureAimAndEject(superstructure, reefLevel, coralObjective, eject, () -> false);
  }

  private static Command superstructureGetBack(
      Superstructure superstructure,
      Supplier<SuperstructureState> holdState,
      BooleanSupplier disableReefAutoAlign) {
    return superstructure
        .runGoal(holdState)
        .until(
            () ->
                superstructure.hasAlgae()
                    ? outOfDistanceToReef(
                        RobotState.getInstance().getEstimatedPose(),
                        minDistanceReefClearAlgae.get())
                    : outOfDistanceToReef(
                        RobotState.getInstance().getEstimatedPose(), minDistanceReefClear.get()))
        .withName("Superstructure Get Back!");
  }

  /** Get drive target. */
  public static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
    var offset = robot.relativeTo(goal);
    double yDistance = Math.abs(offset.getY());
    double xDistance = Math.abs(offset.getX());
    double shiftXT =
        MathUtil.clamp(
            (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
            0.0,
            1.0);
    double shiftYT =
        MathUtil.clamp(yDistance <= 0.2 ? 0.0 : offset.getX() / Reef.faceLength, 0.0, 1.0);
    return goal.transformBy(
        GeomUtil.toTransform2d(
            -shiftXT * maxDistanceReefLineup.get(),
            Math.copySign(shiftYT * maxDistanceReefLineup.get() * 0.8, offset.getY())));
  }

  /** Get position of robot aligned with branch for selected objective. */
  public static Pose2d getCoralScorePose(CoralObjective coralObjective) {
    return getBranchPose(coralObjective)
        .transformBy(DispenserPose.forCoralScore(coralObjective.reefLevel()).toRobotPose());
  }

  public static Pose2d getReefIntakePose(AlgaeObjective objective) {
    int branchId = objective.id() * 2;
    return getBranchPose(new CoralObjective(branchId, ReefLevel.L3))
        .interpolate(getBranchPose(new CoralObjective(branchId + 1, ReefLevel.L3)), 0.5)
        .transformBy(DispenserPose.forAlgaeIntake(objective).toRobotPose());
  }

  private static Pose2d getL1Pose(CoralObjective coralObjective) {
    int face = coralObjective.branchId() / 2;
    return Reef.centerFaces[face].transformBy(
        new Transform2d(
            l1AlignOffsetX.get(),
            l1AlignOffsetY.get() * (coralObjective.branchId() % 2 == 0 ? 1.0 : -1.0),
            Rotation2d.fromDegrees(
                l1AlignOffsetDegrees.get() * (coralObjective.branchId() % 2 == 0 ? 1.0 : -1.0))));
  }

  public static boolean withinDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter <= reefRadius + DriveConstants.robotWidth / 2.0 + distance;
  }

  public static boolean outOfDistanceToReef(Pose2d robot, double distance) {
    final double distanceToReefCenter =
        AllianceFlipUtil.apply(robot).getTranslation().getDistance(Reef.center);
    Logger.recordOutput("AutoScore/DistanceToReefCenter", distanceToReefCenter);
    return distanceToReefCenter >= reefRadius + DriveConstants.robotWidth / 2.0 + distance;
  }

  public static Pose2d getRobotPose(CoralObjective coralObjective) {
    return RobotState.getInstance()
        .getReefPose(coralObjective.branchId() / 2, getCoralScorePose(coralObjective));
  }

  private static Pose2d getRobotPose(AlgaeObjective algaeObjective) {
    return RobotState.getInstance()
        .getReefPose(algaeObjective.id(), getReefIntakePose(algaeObjective));
  }

  public static Pose2d getBranchPose(CoralObjective objective) {
    return Reef.branchPositions2d.get(objective.branchId()).get(objective.reefLevel());
  }
}
