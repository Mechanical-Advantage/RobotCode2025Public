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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
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
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructurePose.CoralDispenserPose;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.Container;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class AutoScoreCommands {
  // Radius of regular hexagon is side length
  private static final double reefRadius = Reef.faceLength;
  private static final double controllerRumbleSecs = 0.2;

  private static final LoggedTunableNumber maxDistanceReefLineup =
      new LoggedTunableNumber("AutoScore/MaxDistanceReefLineup", 1.5);
  public static final LoggedTunableNumber minDistanceReefClearL4 =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClear", Units.inchesToMeters(6.0));
  public static final LoggedTunableNumber minDistanceReefClearAlgaeL4 =
      new LoggedTunableNumber("AutoScore/MinDistanceReefClearAlgae", Units.inchesToMeters(8.0));
  public static final LoggedTunableNumber minAngleReefClear =
      new LoggedTunableNumber("AutoScore/MinAngleReefClear", 60.0);
  public static final LoggedTunableNumber algaeBackupTime =
      new LoggedTunableNumber("AutoScore/AlgaeBackupTime", 0.5);
  private static final LoggedTunableNumber distanceSuperstructureReady =
      new LoggedTunableNumber("AutoScore/DistanceSuperstructureReady", 3);
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
  private static final LoggedTunableNumber thetaToleranceReady =
      new LoggedTunableNumber("AutoScore/ThetaToleranceReady", 35.0);
  private static final LoggedTunableNumber thetaToleranceEject =
      new LoggedTunableNumber("AutoScore/ThetaToleranceEject", 2.0);
  private static final LoggedTunableNumber l2ReefIntakeDistance =
      new LoggedTunableNumber("AutoScore/L2ReefIntakeDistance", 0.12);
  private static final LoggedTunableNumber l3ReefIntakeDistance =
      new LoggedTunableNumber("AutoScore/L3ReefIntakeDistance", 0.14);
  private static final LoggedTunableNumber l1AlignOffsetX =
      new LoggedTunableNumber("AutoScore/L1AlignOffsetX", 0.45);
  private static final LoggedTunableNumber l1AlignOffsetY =
      new LoggedTunableNumber(
          "AutoScore/L1AlignOffsetY",
          FieldConstants.Reef.faceLength / 2.0 - Units.inchesToMeters(2.5));
  private static final LoggedTunableNumber minDistanceAim =
      new LoggedTunableNumber("AutoScore/MinDistanceAim", 0.2);
  private static final LoggedTunableNumber ejectTimeSeconds =
      new LoggedTunableNumber("AutoScore/EjectTimeSeconds", 0.5);
  private static final LoggedTunableNumber correctiveMeasureFF =
      new LoggedTunableNumber("AutoScore/CorrectiveMeasureFF", 0.7);
  private static final LoggedTunableNumber correctiveMeasureDistance =
      new LoggedTunableNumber("AutoScore/CorrectiveMeasureDistance", Units.inchesToMeters(20.0));
  private static final LoggedTunableNumber correctiveMeasureBlendDistance =
      new LoggedTunableNumber(
          "AutoScore/CorrectiveMeasureBlendDistance", Units.inchesToMeters(12.0));

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
    Supplier<Pose2d> robot =
        () ->
            coralObjective
                .get()
                .map(objective -> getRobotPose(objective, superstructure.hasAlgae()))
                .orElseGet(() -> RobotState.getInstance().getEstimatedPose());

    Function<CoralObjective, Pose2d> goal =
        objective ->
            objective.reefLevel() == ReefLevel.L1
                ? getL1Pose(objective)
                : getCoralScorePose(objective, false);

    Container<CoralObjective> coralObjectiveScored = new Container<>();
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
                    .map(
                        objective -> {
                          boolean hasAlgae = superstructure.hasAlgae();
                          if (reefLevel.get() == ReefLevel.L1) {
                            return getDriveTarget(
                                robot.get(), AllianceFlipUtil.apply(getL1Pose(objective)));
                          }
                          Pose2d goalPose = getCoralScorePose(objective, hasAlgae);
                          final double clearDistance =
                              hasAlgae
                                  ? minDistanceReefClearAlgaeL4.get()
                                  : minDistanceReefClearL4.get();
                          if ((!superstructure.readyForL4() && reefLevel.get() == ReefLevel.L4)
                              || (superstructure.readyForL4()
                                  && withinDistanceToReef(robot.get(), clearDistance - 0.05)
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
                          return new Pose2d(
                              getDriveTarget(robot.get(), AllianceFlipUtil.apply(goalPose))
                                  .getTranslation(),
                              AllianceFlipUtil.apply(getBranchPose(objective))
                                  .getTranslation()
                                  .minus(robot.get().getTranslation())
                                  .getAngle());
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
        .and(() -> !disableReefAutoAlign.getAsBoolean())
        .onTrue(
            getBackCorrectiveMeasure(drive, driverX, driverY, driverOmega, robotRelative)
                .finallyDo(() -> needsToGetBack.value = false)
                .withName("Corrective Measure"));

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
            preIntake(
                superstructure,
                funnel,
                robot,
                () -> reefLevel.get() == ReefLevel.L4,
                disableReefAutoAlign),
            // Check if need wait until pre ready or already ready
            Commands.waitUntil(
                () -> {
                  boolean ready =
                      (withinDistanceToReef(robot.get(), distanceSuperstructureReady.get())
                              && coralObjective.get().isPresent()
                              && Math.abs(
                                      AllianceFlipUtil.apply(robot.get())
                                          .relativeTo(goal.apply(coralObjective.get().get()))
                                          .getRotation()
                                          .getDegrees())
                                  <= thetaToleranceReady.get())
                          || disableReefAutoAlign.getAsBoolean();
                  Logger.recordOutput("AutoScore/AllowPreReady", ready);

                  // Get back!
                  if (ready
                      && reefLevel.get() == ReefLevel.L4
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
                                          RobotState.getInstance()
                                              .getRobotVelocity()
                                              .vxMetersPerSecond,
                                          RobotState.getInstance()
                                              .getRobotVelocity()
                                              .vyMetersPerSecond)
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

    // Schedule get back command
    if (funnel.isEmpty()) {
      new Trigger(() -> hasEnded.value && needsToGetBack.value)
          .and(() -> !disableReefAutoAlign.getAsBoolean())
          .onTrue(
              getBackCorrectiveMeasure(drive, driverX, driverY, driverOmega, robotRelative)
                  .finallyDo(() -> needsToGetBack.value = false)
                  .withName("Corrective Measure"));
    }

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
                                      (withinDistanceToReef(
                                                  robot.get(), distanceSuperstructureReady.get())
                                              && outOfDistanceToReef(
                                                  robot.get(), minDistanceReefClearL4.get() - 0.1)
                                              && algaeObjective.get().isPresent()
                                              && Math.abs(
                                                      AllianceFlipUtil.apply(robot.get())
                                                          .relativeTo(goal.get())
                                                          .getRotation()
                                                          .getDegrees())
                                                  <= thetaToleranceReady.get())
                                          || disableReefAutoAlign.getAsBoolean();
                                  Logger.recordOutput("AutoScore/AllowPreReady", ready);

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
              Leds.getInstance().superAutoScoring = true;
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
                        ejectTimer.hasElapsed(ejectTimeSeconds.get())
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
                                        .transformBy(
                                            GeomUtil.toTransform2d(
                                                getCoralDispenserPose(
                                                                objective.reefLevel(),
                                                                superstructure.hasAlgae())
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
    return superstructureAimAndEject(
        superstructure, reefLevel, coralObjective, eject, () -> false, () -> false);
  }

  private static Command preIntake(
      Superstructure superstructure,
      RollerSystem funnel,
      Supplier<Pose2d> robot,
      BooleanSupplier shouldClearReef,
      BooleanSupplier disableReefAutoAlign) {
    return IntakeCommands.intake(superstructure, funnel)
        .until(superstructure::hasCoral)
        .onlyIf(
            () ->
                !superstructure.hasCoral()
                    && ((outOfDistanceToReef(robot.get(), minDistanceReefClearL4.get())
                            && shouldClearReef.getAsBoolean())
                        || disableReefAutoAlign.getAsBoolean()));
  }

  private static Command getBackCorrectiveMeasure(
      Drive drive,
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      BooleanSupplier robotRelative) {
    Supplier<Translation2d> correctiveMeasure =
        () -> {
          final Pose2d flippedRobot =
              AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose());
          Translation2d driverLinearVel =
              DriveCommands.getLinearVelocityFromJoysticks(
                  driverX.getAsDouble(), driverY.getAsDouble());

          return driverLinearVel.plus(
              flippedRobot
                  .getTranslation()
                  .minus(Reef.center)
                  .div(flippedRobot.getTranslation().getDistance(Reef.center))
                  .times(
                      correctiveMeasureFF.get()
                          * (1.0
                              - MathUtil.clamp(
                                  (flippedRobot.getTranslation().getDistance(Reef.center)
                                          - reefRadius
                                          - DriveConstants.robotWidth / 2.0
                                          - correctiveMeasureBlendDistance.get())
                                      / (correctiveMeasureDistance.get()
                                          - correctiveMeasureBlendDistance.get()),
                                  0.0,
                                  1.0)))
                  .times(driverLinearVel.getNorm()));
        };
    return DriveCommands.joystickDrive(
            drive,
            () -> correctiveMeasure.get().getX(),
            () -> correctiveMeasure.get().getY(),
            driverOmega,
            robotRelative)
        .until(
            () ->
                outOfDistanceToReef(
                    RobotState.getInstance().getEstimatedPose(), correctiveMeasureDistance.get()));
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
    if (Math.abs(angleToGoal.getDegrees()) >= 30.0 && withinDistanceToReef(robot, 1)) {
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
              -shiftXT * maxDistanceReefLineup.get(),
              Math.copySign(shiftYT * maxDistanceReefLineup.get() * 0.8, offset.getY())));
    }
    return goal.transformBy(
        GeomUtil.toTransform2d(
            -MathUtil.clamp(
                    (robot.getTranslation().getDistance(goal.getTranslation()) - 1.0) / 1.2,
                    0.0,
                    1.0)
                * 0.7,
            0.0));
  }

  /** Get position of robot aligned with branch for selected objective. */
  public static Pose2d getCoralScorePose(CoralObjective coralObjective, boolean algae) {
    return getBranchPose(coralObjective)
        .transformBy(getCoralDispenserPose(coralObjective.reefLevel(), algae).toRobotPose());
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
            new Transform2d(l1AlignOffsetX.get(), 0, Rotation2d.kPi));
    return RobotState.getInstance()
        .getEstimatedPose()
        .nearest(
            List.of(
                centerFace.transformBy(new Transform2d(0, -l1AlignOffsetY.get(), Rotation2d.kZero)),
                centerFace,
                centerFace.transformBy(
                    new Transform2d(0, l1AlignOffsetY.get(), Rotation2d.kZero))));
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

  public static Pose2d getRobotPose(CoralObjective coralObjective, boolean algae) {
    return RobotState.getInstance()
        .getReefPose(coralObjective.branchId() / 2, getCoralScorePose(coralObjective, algae));
  }

  private static Pose2d getRobotPose(AlgaeObjective algaeObjective) {
    return RobotState.getInstance()
        .getReefPose(algaeObjective.id(), getReefIntakePose(algaeObjective));
  }

  public static Pose2d getBranchPose(CoralObjective objective) {
    return Reef.branchPositions2d.get(objective.branchId()).get(objective.reefLevel());
  }

  private static CoralDispenserPose getCoralDispenserPose(ReefLevel reefLevel, boolean algae) {
    return switch (reefLevel) {
      case L1, L2 -> algae ? CoralDispenserPose.ALGAE_L2 : CoralDispenserPose.L2;
      case L3 -> algae ? CoralDispenserPose.ALGAE_L3 : CoralDispenserPose.L3;
      case L4 -> algae ? CoralDispenserPose.ALGAE_L4 : CoralDispenserPose.L4;
    };
  }
}
