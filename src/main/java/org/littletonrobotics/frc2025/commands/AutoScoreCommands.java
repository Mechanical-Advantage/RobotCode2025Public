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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.*;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.FieldConstants.AlgaeObjective;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.FieldConstants.Reef;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.RobotState.ReefPoseEstimate;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructurePose.CoralDispenserPose;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.util.*;
import org.littletonrobotics.junction.Logger;

public class AutoScoreCommands {
  // Radius of regular hexagon is side length
  private static final double reefRadius = Reef.faceLength;
  private static final double controllerRumbleSecs = 0.2;

  private static final LoggedTunableNumber maxDistanceReefLineupX =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineupX", 1.0);
  private static final LoggedTunableNumber maxDistanceReefLineupY =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineupY", 1.2);
  public static final LoggedTunableNumber minDistanceReefClearL4 =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClear", Units.inchesToMeters(4.0));
  public static final LoggedTunableNumber minDistanceReefClearAlgaeL4 =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClearAlgae", 0.45);
  public static final LoggedTunableNumber minAngleReefClear =
      new LoggedTunableNumber("AutoScore/MinAngleReefClear", 30.0);
  public static final LoggedTunableNumber algaeBackupTime =
      new LoggedTunableNumber("AutoScore/AlgaeBackupTime", 0.5);
  private static final LoggedTunableNumber distanceSuperstructureReady =
      new LoggedTunableNumber("AutoScore/DistanceSuperstructureReady", 2);
  private static final LoggedTunableNumber distanceSuperstructureReadyAuto =
      new LoggedTunableNumber("AutoScore/DistanceSuperstructureReadyAuto", 2.0);
  private static final LoggedTunableNumber thetaToleranceReady =
      new LoggedTunableNumber("AutoScore/ThetaToleranceReady", 35.0);
  private static final LoggedTunableNumber arcDistanceReady =
      new LoggedTunableNumber("AutoScore/ArcDistanceReady", 0.7);
  private static final LoggedTunableNumber arcDistanceReadyAuto =
      new LoggedTunableNumber("AutoScore/ArcDistanceReadyAuto", 1.5);
  private static final LoggedTunableNumber[] lowerXToleranceEject = {
    new LoggedTunableNumber("AutoScore/XToleranceEject/Lower/L1", 0.03),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Lower/L2", 0.05),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Lower/L3", 0.05),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Lower/L4", 0.05)
  };
  private static final LoggedTunableNumber[] upperXToleranceEject = {
    new LoggedTunableNumber("AutoScore/XToleranceEject/Upper/L1", 0.03),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Upper/L2", 0.1),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Upper/L3", 0.1),
    new LoggedTunableNumber("AutoScore/XToleranceEject/Upper/L4", 0.1)
  };
  private static final LoggedTunableNumber[] yToleranceEject = {
    new LoggedTunableNumber("AutoScore/YToleranceEject/L1", 0.05),
    new LoggedTunableNumber("AutoScore/YToleranceEject/L2", 0.05),
    new LoggedTunableNumber("AutoScore/YToleranceEject/L3", 0.05),
    new LoggedTunableNumber("AutoScore/YToleranceEject/L4", 0.05)
  };
  private static final LoggedTunableNumber[] lookaheadEject = {
    new LoggedTunableNumber("AutoScore/LookaheadToleranceEject/L1", 0.5),
    new LoggedTunableNumber("AutoScore/LookaheadToleranceEject/L2", 0.5),
    new LoggedTunableNumber("AutoScore/LookaheadToleranceEject/L3", 0.5),
    new LoggedTunableNumber("AutoScore/LookaheadToleranceEject/L4", 0.3)
  };
  private static final LoggedTunableNumber l1ThetaEject =
      new LoggedTunableNumber("AutoScore/L1ThetaToleranceEject", 5.0);
  private static final LoggedTunableNumber minBlendTransform =
      new LoggedTunableNumber("AutoScore/MinBlendTransform", 0.8);
  private static final LoggedTunableNumber singleTagTransform =
      new LoggedTunableNumber("AutoScore/SingleTagTransform", 0.3);
  private static final LoggedTunableNumber l4EjectDelay =
      new LoggedTunableNumber("AutoScore/L4EjectDelay", 0.05);
  private static final LoggedTunableNumber l4EjectDelayAuto =
      new LoggedTunableNumber("AutoScore/L4EjectDelayAuto", 0.05);
  private static final LoggedTunableNumber l2ReefIntakeDistance =
      new LoggedTunableNumber("AutoScore/L2ReefIntakeDistance", 0.12);
  private static final LoggedTunableNumber l3ReefIntakeDistance =
      new LoggedTunableNumber("AutoScore/L3ReefIntakeDistance", 0.14);
  private static final LoggedTunableNumber maxAimingAngle =
      new LoggedTunableNumber("AutoScore/MaxAimingAngle", 20.0);
  private static final LoggedTunableNumber l1AlignOffsetX =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetX", 0.45);
  private static final LoggedTunableNumber l1AlignOffsetY =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetY", 0.0);
  private static final LoggedTunableNumber l1AlignOffsetTheta =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetTheta", 180.0);
  private static final LoggedTunableNumber minDistanceAim =
      new LoggedTunableNumber("AutoScore/MinDistanceAim", 0.2);
  private static final LoggedTunableNumber[] ejectTimeSeconds = {
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L1", 0.5),
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L2", 0.5),
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L3", 0.5),
    new LoggedTunableNumber("AutoScore/EjectTimeSeconds/L4", 0.3)
  };
  private static final LoggedTunableNumber[] branchFudgeX = {
    new LoggedTunableNumber("AutoScore/FudgeX/L1", 0.0),
    new LoggedTunableNumber("AutoScore/FudgeX/L2", 0.0),
    new LoggedTunableNumber("AutoScore/FudgeX/L3", 0.0),
    new LoggedTunableNumber("AutoScore/FudgeX/L4", 0.0)
  };
  private static final boolean enableAimingInAuto = true;

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
      Command controllerRumble,
      BooleanSupplier robotRelative,
      BooleanSupplier disableReefAutoAlign,
      BooleanSupplier manualEject) {
    Supplier<ReefPoseEstimate> robot =
        () ->
            coralObjective
                .get()
                .map(objective -> getRobotPose(objective, superstructure.hasAlgae()))
                .orElseGet(
                    () -> new ReefPoseEstimate(RobotState.getInstance().getEstimatedPose(), 0.0));

    Function<CoralObjective, Pose2d> goal =
        objective -> {
          boolean hasAlgae = superstructure.hasAlgae();
          if (reefLevel.get() == ReefLevel.L1) {
            return AllianceFlipUtil.apply(getL1Pose(objective));
          }
          Pose2d goalPose = getCoralScorePose(objective, hasAlgae);
          var reefPoseEstimate = robot.get();
          final double clearDistance =
              hasAlgae ? minDistanceReefClearAlgaeL4.get() : minDistanceReefClearL4.get();
          if (reefPoseEstimate.blend() <= minBlendTransform.get()
              && Constants.getRobot() != Constants.RobotType.SIMBOT) {
            goalPose = goalPose.transformBy(GeomUtil.toTransform2d(-singleTagTransform.get(), 0.0));
          } else if ((!superstructure.readyForL4() && reefLevel.get() == ReefLevel.L4)
              || (superstructure.readyForL4()
                  && withinDistanceToReef(reefPoseEstimate.pose(), clearDistance - 0.05)
                  && reefLevel.get() != ReefLevel.L4)) {
            goalPose =
                goalPose.transformBy(
                    GeomUtil.toTransform2d(
                        goalPose.getTranslation().getDistance(Reef.center)
                            - reefRadius
                            - (DriveConstants.robotWidth / 2.0)
                            - clearDistance,
                        0.0));
          }
          Logger.recordOutput("AutoScore/Goal", AllianceFlipUtil.apply(goalPose));
          if (DriverStation.isAutonomousEnabled() && !enableAimingInAuto) {
            return AllianceFlipUtil.apply(goalPose);
          } else {
            Pose2d flippedGoalPose = AllianceFlipUtil.apply(goalPose);
            Rotation2d originalRotation = flippedGoalPose.getRotation();
            Rotation2d rotationAdjustment =
                AllianceFlipUtil.apply(getBranchPose(objective))
                    .getTranslation()
                    .minus(reefPoseEstimate.pose().getTranslation())
                    .getAngle()
                    .minus(originalRotation);
            boolean isLeftBranch = objective.branchId() % 2 == 0;
            rotationAdjustment =
                Rotation2d.fromDegrees(
                    MathUtil.clamp(
                        rotationAdjustment.getDegrees(),
                        isLeftBranch ? 0.0 : -maxAimingAngle.get(),
                        isLeftBranch ? maxAimingAngle.get() : 0.0));
            return new Pose2d(
                flippedGoalPose.getTranslation(), originalRotation.plus(rotationAdjustment));
          }
        };

    Container<Boolean> needsToGetBack = new Container<>(false);
    Container<Boolean> hasEnded = new Container<>(false);

    var driveCommand =
        new DriveToPose(
            drive,
            () ->
                coralObjective
                    .get()
                    .filter(
                        objective ->
                            !(superstructure.hasAlgae() && objective.reefLevel() == ReefLevel.L1))
                    .map(objective -> getDriveTarget(robot.get().pose(), goal.apply(objective)))
                    .orElseGet(() -> RobotState.getInstance().getEstimatedPose()),
            () -> robot.get().pose(),
            () ->
                DriveCommands.getLinearVelocityFromJoysticks(
                        driverX.getAsDouble(), driverY.getAsDouble())
                    .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
            () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble()));

    Timer l4EjectTimer = new Timer();
    l4EjectTimer.start();
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
              Logger.recordOutput("AutoScore/AllowReady", false);
              Logger.recordOutput("AutoScore/AllowEject", false);
            })
        .andThen(
            // Run superstructure
            preIntake(
                superstructure,
                funnel,
                () -> robot.get().pose(),
                () -> reefLevel.get() == ReefLevel.L4,
                disableReefAutoAlign),
            // Check if need wait until pre ready or already ready
            Commands.waitUntil(
                () -> {
                  boolean ready =
                      coralObjective.get().isPresent()
                              && readyForSuperstructure(
                                  robot.get().pose(),
                                  goal.apply(coralObjective.get().get()),
                                  reefLevel.get() == ReefLevel.L4)
                          || disableReefAutoAlign.getAsBoolean();
                  Logger.recordOutput("AutoScore/AllowReady", ready);

                  // Get back!
                  if (ready
                      && (reefLevel.get() == ReefLevel.L4 || superstructure.hasAlgae())
                      && !disableReefAutoAlign.getAsBoolean()
                      && DriverStation.isTeleopEnabled()) {
                    needsToGetBack.value = true;
                    superstructure.setReefDangerState(
                        Optional.of(
                            Superstructure.getScoringState(
                                reefLevel.get(), superstructure.hasAlgae(), false)));
                  }
                  return ready;
                }),
            superstructureAimAndEject(
                    superstructure,
                    reefLevel,
                    coralObjective,
                    () -> {
                      if (coralObjective.get().isEmpty()) return false;
                      var reefPose = robot.get();

                      var objective = coralObjective.get().get();
                      Pose2d flippedRobot = AllianceFlipUtil.apply(reefPose.pose());
                      Pose2d predictedRobot =
                          flippedRobot.exp(
                              RobotState.getInstance()
                                  .getRobotVelocity()
                                  .toTwist2d(lookaheadEject[reefLevel.get().ordinal()].get()));

                      boolean ready =
                          (EqualsUtil.epsilonEquals(reefPose.blend(), 1.0)
                                      || Constants.getRobot() == Constants.RobotType.SIMBOT)
                                  && checkEjectTolerances(
                                      flippedRobot,
                                      objective,
                                      reefLevel.get(),
                                      superstructure.hasAlgae())
                                  && checkEjectTolerances(
                                      predictedRobot,
                                      objective,
                                      reefLevel.get(),
                                      superstructure.hasAlgae())
                                  && superstructure.atGoal()
                                  && !disableReefAutoAlign.getAsBoolean()
                              || manualEject.getAsBoolean();
                      if (reefLevel.get() == ReefLevel.L4) {
                        if (!ready) {
                          l4EjectTimer.restart();
                        }
                        ready =
                            ready
                                && l4EjectTimer.hasElapsed(
                                    DriverStation.isAutonomous()
                                        ? l4EjectDelayAuto.get()
                                        : l4EjectDelay.get());
                      }
                      Logger.recordOutput("AutoScore/AllowEject", ready);
                      return ready;
                    },
                    manualEject,
                    disableReefAutoAlign)
                .andThen(
                    new ScheduleCommand(controllerRumble.withTimeout(controllerRumbleSecs)),
                    superstructure
                        .runGoal(
                            () ->
                                Superstructure.getScoringState(
                                    reefLevel.get(), superstructure.hasAlgae(), false))
                        .until(() -> !disableReefAutoAlign.getAsBoolean())))
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
        Commands.none(),
        () -> false,
        () -> false,
        () -> false);
  }

  public static Command reefIntake(
      Drive drive,
      Superstructure superstructure,
      Optional<RollerSystem> funnel,
      Supplier<Optional<AlgaeObjective>> algaeObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      Command controllerRumble,
      BooleanSupplier robotRelative,
      BooleanSupplier disableReefAutoAlign) {
    Supplier<Pose2d> robot =
        () ->
            algaeObjective
                .get()
                .map(objective -> AutoScoreCommands.getRobotPose(objective).pose())
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
    Container<Boolean> complete = new Container<>(false);

    Timer hasAlgaeTimer = new Timer();
    hasAlgaeTimer.start();
    Supplier<Pose2d> goal =
        () ->
            algaeObjective
                .get()
                .map(AutoScoreCommands::getReefIntakePose)
                .orElseGet(
                    () ->
                        algaeIntaked.value == null
                            ? AllianceFlipUtil.apply(robot.get())
                            : getReefIntakePose(algaeIntaked.value));

    return Commands.runOnce(
            () -> {
              // Reset State
              algaeIntaked.value = null;
              needsToGetBack.value = false;
              hasEnded.value = false;
              complete.value = false;
            })
        .andThen(
            Commands.either(
                    joystickDrive,
                    new DriveToPose(
                        drive,
                        () -> {
                          Pose2d goalPose = goal.get();
                          if (algaeObjective.get().isEmpty() && !superstructure.hasAlgae()) {
                            return AllianceFlipUtil.apply(goalPose);
                          }
                          if (superstructure.getState() != algaeIntakeState.get()
                              && algaeObjective.get().isPresent()) {
                            goalPose =
                                goalPose.transformBy(
                                    GeomUtil.toTransform2d(
                                        -minDistanceReefClearAlgaeL4.get(), 0.0));
                          }
                          if (superstructure.hasAlgae()) {
                            if (!disableReefAutoAlign.getAsBoolean()) {
                              RobotState.getInstance()
                                  .setDistanceToReefAlgae(
                                      OptionalDouble.of(
                                          AllianceFlipUtil.apply(
                                                      robot
                                                          .get()
                                                          .transformBy(
                                                              GeomUtil.toTransform2d(
                                                                  SuperstructureConstants
                                                                          .dispenserOrigin2d
                                                                          .getX()
                                                                      + algaeIntakeState
                                                                              .get()
                                                                              .getValue()
                                                                              .getPose()
                                                                              .elevatorHeight()
                                                                              .getAsDouble()
                                                                          * SuperstructureConstants
                                                                              .elevatorAngle
                                                                              .getCos(),
                                                                  0.0)))
                                                  .getTranslation()
                                                  .getDistance(Reef.center)
                                              - reefRadius));
                            }
                            if (hasAlgaeTimer.hasElapsed(algaeBackupTime.get())
                                && !disableReefAutoAlign.getAsBoolean()) {
                              complete.value = true;
                              RobotState.getInstance()
                                  .setDistanceToReefAlgae(OptionalDouble.empty());
                            }
                            goalPose =
                                goalPose.transformBy(
                                    GeomUtil.toTransform2d(
                                        -minDistanceReefClearAlgaeL4.get()
                                            * Math.min(
                                                1.0, hasAlgaeTimer.get() / algaeBackupTime.get()),
                                        0.0));
                          } else {
                            RobotState.getInstance().setDistanceToReefAlgae(OptionalDouble.empty());
                            hasAlgaeTimer.restart();
                          }
                          return getDriveTarget(robot.get(), AllianceFlipUtil.apply(goalPose));
                        },
                        robot,
                        () ->
                            DriveCommands.getLinearVelocityFromJoysticks(
                                    driverX.getAsDouble(), driverY.getAsDouble())
                                .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
                        () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble())),
                    disableReefAutoAlign)
                .alongWith(
                    (funnel.isEmpty()
                            ? Commands.none()
                            : preIntake(
                                superstructure,
                                funnel.get(),
                                robot,
                                () -> false,
                                disableReefAutoAlign))
                        .andThen(
                            // Check if need wait until pre ready or already ready
                            Commands.waitUntil(
                                () -> {
                                  boolean ready =
                                      readyForSuperstructure(
                                                  robot.get(),
                                                  AllianceFlipUtil.apply(goal.get()),
                                                  false)
                                              && algaeObjective.get().isPresent()
                                          || disableReefAutoAlign.getAsBoolean();
                                  Logger.recordOutput("ReefIntake/AllowReady", ready);
                                  // Get back!
                                  if (ready && DriverStation.isTeleopEnabled()) {
                                    needsToGetBack.value = true;
                                  }
                                  return ready;
                                }),
                            superstructure
                                .runGoal(algaeIntakeState)
                                .alongWith(
                                    Commands.waitUntil(
                                            () ->
                                                superstructure.getState() == algaeIntakeState.get())
                                        .andThen(
                                            () ->
                                                superstructure.setReefDangerState(
                                                    disableReefAutoAlign.getAsBoolean()
                                                        ? Optional.empty()
                                                        : Optional.of(algaeIntakeState.get()))))),
                    Commands.waitUntil(
                            () -> superstructure.hasAlgae() && algaeObjective.get().isPresent())
                        .andThen(
                            Commands.runOnce(
                                () -> {
                                  algaeIntaked.value = algaeObjective.get().get();
                                }))))
        .until(() -> complete.value)
        .finallyDo(
            () -> {
              complete.value = false;
              hasEnded.value = true;
            })
        .deadlineFor(Commands.run(() -> Logger.recordOutput("ReefIntake/Complete", complete.value)))
        .andThen(new ScheduleCommand(controllerRumble.withTimeout(controllerRumbleSecs)));
  }

  public static Command reefIntake(
      Drive drive,
      Superstructure superstructure,
      Supplier<Optional<AlgaeObjective>> algaeObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Command joystickDrive,
      Command controllerRumble,
      BooleanSupplier robotRelative,
      BooleanSupplier disableReefAutoAlign) {
    return reefIntake(
        drive,
        superstructure,
        Optional.empty(),
        algaeObjective,
        driverX,
        driverY,
        driverOmega,
        joystickDrive,
        controllerRumble,
        robotRelative,
        disableReefAutoAlign);
  }

  public static Command superAutoScore(
      Drive drive,
      Superstructure superstructure,
      RollerSystem funnel,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      Supplier<Command> joystickDriveCommandFactory,
      Supplier<Command> controllerRumbleCommandFactory,
      BooleanSupplier robotRelative,
      BooleanSupplier disableReefAutoAlign,
      BooleanSupplier manualEject) {

    return reefIntake(
            drive,
            superstructure,
            Optional.of(funnel),
            () ->
                coralObjective.get().map(objective -> new AlgaeObjective(objective.branchId() / 2)),
            driverX,
            driverY,
            driverOmega,
            joystickDriveCommandFactory.get(),
            controllerRumbleCommandFactory.get(),
            robotRelative,
            disableReefAutoAlign)
        .andThen(
            autoScore(
                drive,
                superstructure,
                funnel,
                reefLevel,
                coralObjective,
                driverX,
                driverY,
                driverOmega,
                joystickDriveCommandFactory.get(),
                controllerRumbleCommandFactory.get(),
                robotRelative,
                disableReefAutoAlign,
                manualEject))
        .beforeStarting(
            () -> {
              Leds.getInstance().autoScoringReef = true;
              Leds.getInstance().superAutoScoring = true;
              Leds.getInstance().autoScoringLevel = reefLevel.get();
            })
        .finallyDo(
            () -> {
              Leds.getInstance().autoScoringReef = false;
              Leds.getInstance().superAutoScoring = false;
            });
  }

  public static Command superstructureAimAndEject(
      Superstructure superstructure,
      Supplier<ReefLevel> reefLevel,
      Supplier<Optional<CoralObjective>> coralObjective,
      BooleanSupplier eject,
      BooleanSupplier forceEject,
      BooleanSupplier disableReefAutoAlign) {
    final Timer ejectTimer = new Timer();
    return superstructure
        .runGoal(
            () -> Superstructure.getScoringState(reefLevel.get(), superstructure.hasAlgae(), false))
        .until(eject)
        .andThen(
            Commands.runOnce(ejectTimer::restart),
            superstructure
                .runGoal(
                    () ->
                        Superstructure.getScoringState(
                            reefLevel.get(), superstructure.hasAlgae(), true))
                .until(
                    () ->
                        ejectTimer.hasElapsed(ejectTimeSeconds[reefLevel.get().levelNumber].get())
                            && !forceEject.getAsBoolean()))
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
                                    getRobotPose(objective, superstructure.hasAlgae())
                                        .pose()
                                        .transformBy(
                                            getCoralDispenserPose(
                                                    objective.reefLevel(),
                                                    superstructure.hasAlgae())
                                                .robotToDispenser()));
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
    return superstructureAimAndEject(
        superstructure, reefLevel, coralObjective, eject, () -> false, () -> false);
  }

  private static Command preIntake(
      Superstructure superstructure,
      RollerSystem funnel,
      Supplier<Pose2d> robot,
      BooleanSupplier shouldClearReef,
      BooleanSupplier disableReefAutoAlign) {
    return Commands.waitUntil(
            () ->
                outOfDistanceToReef(robot.get(), minDistanceReefClearL4.get())
                    || !shouldClearReef.getAsBoolean()
                    || disableReefAutoAlign.getAsBoolean())
        .andThen(IntakeCommands.intake(superstructure, funnel).until(superstructure::hasCoral))
        .onlyIf(() -> !superstructure.hasCoral());
  }

  /** Get drive target. */
  public static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
    Rotation2d angleToGoal =
        robot
            .getTranslation()
            .minus(AllianceFlipUtil.apply(Reef.center))
            .getAngle()
            .minus(goal.getTranslation().minus(AllianceFlipUtil.apply(Reef.center)).getAngle());
    Logger.recordOutput("AutoScore/AngleToGoal", angleToGoal);
    var offset = robot.relativeTo(goal);
    double yDistance = Math.abs(offset.getY());
    double xDistance = Math.abs(offset.getX());
    double shiftXT =
        MathUtil.clamp(
            (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 4)),
            0.0,
            1.0);
    double shiftYT =
        MathUtil.clamp(yDistance <= 0.2 ? 0.0 : offset.getX() / Reef.faceLength, 0.0, 1.0);
    return goal.transformBy(
        GeomUtil.toTransform2d(
            -shiftXT * maxDistanceReefLineupX.get(),
            Math.copySign(shiftYT * maxDistanceReefLineupY.get(), offset.getY())));
  }

  /** Get position of robot aligned with branch for selected objective. */
  public static Pose2d getCoralScorePose(CoralObjective coralObjective, boolean algae) {
    return getBranchPose(coralObjective)
        .transformBy(getCoralDispenserPose(coralObjective.reefLevel(), algae).branchToRobot());
  }

  public static Pose2d getReefIntakePose(AlgaeObjective objective) {
    int branchId = objective.id() * 2;
    return getBranchPose(new CoralObjective(branchId, ReefLevel.L3))
        .interpolate(getBranchPose(new CoralObjective(branchId + 1, ReefLevel.L3)), 0.5)
        .transformBy(
            GeomUtil.toTransform2d(
                (objective.low() ? l2ReefIntakeDistance.get() : l3ReefIntakeDistance.get())
                    + DriveConstants.robotWidth / 2.0,
                0.0))
        .transformBy(GeomUtil.toTransform2d(Rotation2d.kPi));
  }

  private static Pose2d getL1Pose(CoralObjective coralObjective) {
    Pose2d centerFace =
        Reef.centerFaces[coralObjective.branchId() / 2].transformBy(
            new Transform2d(
                l1AlignOffsetX.get(),
                l1AlignOffsetY.get(),
                Rotation2d.fromDegrees(l1AlignOffsetTheta.get())));
    return centerFace;
  }

  private static boolean checkEjectTolerances(
      Pose2d flippedRobot, CoralObjective objective, ReefLevel level, boolean algae) {
    // Separate tolerances for L1
    if (level == ReefLevel.L1) {
      var error = flippedRobot.relativeTo(getL1Pose(objective));
      return Math.abs(error.getX()) <= lowerXToleranceEject[level.ordinal()].get()
          && Math.abs(error.getY()) <= yToleranceEject[level.ordinal()].get()
          && Math.abs(error.getRotation().getDegrees()) <= l1ThetaEject.get();
    }

    var dispenserPose = getCoralDispenserPose(level, algae);
    var error =
        getBranchPose(objective)
            .relativeTo(flippedRobot.transformBy(dispenserPose.robotToDispenser()))
            .getTranslation();
    double rawDistance = error.getNorm();
    double yError = Math.abs(error.getAngle().getTan() * rawDistance);
    double xError =
        Math.abs(rawDistance / error.getAngle().getCos()) - dispenserPose.getPose().get().getX();
    Logger.recordOutput("AutoScore/YError", yError);
    Logger.recordOutput("AutoScore/XError", xError);
    return yError <= yToleranceEject[level.ordinal()].get()
        && xError >= -lowerXToleranceEject[level.ordinal()].get()
        && xError <= upperXToleranceEject[level.ordinal()].get();
  }

  public static boolean readyForSuperstructure(Pose2d robot, Pose2d goal, boolean shouldBackUp) {
    double arcDistance =
        robot
                .getTranslation()
                .minus(AllianceFlipUtil.apply(Reef.center))
                .getAngle()
                .minus(goal.getTranslation().minus(AllianceFlipUtil.apply(Reef.center)).getAngle())
                .getRadians()
            * robot.getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center));
    return withinDistanceToReef(
            robot,
            DriverStation.isAutonomousEnabled()
                ? distanceSuperstructureReadyAuto.get()
                : distanceSuperstructureReady.get())
        && (outOfDistanceToReef(robot, minDistanceReefClearL4.get() - 0.1) || !shouldBackUp)
        && Math.abs(robot.relativeTo(goal).getRotation().getDegrees()) <= thetaToleranceReady.get()
        && Math.abs(arcDistance)
            <= (DriverStation.isAutonomousEnabled()
                ? arcDistanceReadyAuto.get()
                : arcDistanceReady.get());
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

  public static ReefPoseEstimate getRobotPose(CoralObjective coralObjective, boolean algae) {
    return RobotState.getInstance()
        .getReefPose(coralObjective.branchId() / 2, getCoralScorePose(coralObjective, algae));
  }

  private static ReefPoseEstimate getRobotPose(AlgaeObjective algaeObjective) {
    return RobotState.getInstance()
        .getReefPose(algaeObjective.id(), getReefIntakePose(algaeObjective));
  }

  public static Pose2d getBranchPose(CoralObjective objective) {
    return Reef.branchPositions2d
        .get(objective.branchId())
        .get(objective.reefLevel())
        .transformBy(
            new Transform2d(
                branchFudgeX[objective.reefLevel().levelNumber].get(), 0, Rotation2d.kZero));
  }

  private static CoralDispenserPose getCoralDispenserPose(ReefLevel reefLevel, boolean algae) {
    return switch (reefLevel) {
      case L1, L2 -> algae ? CoralDispenserPose.ALGAE_L2 : CoralDispenserPose.L2;
      case L3 -> algae ? CoralDispenserPose.ALGAE_L3 : CoralDispenserPose.L3;
      case L4 -> algae ? CoralDispenserPose.ALGAE_L4 : CoralDispenserPose.L4;
    };
  }
}
