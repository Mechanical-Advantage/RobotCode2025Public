// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.commands;

import static org.littletonrobotics.frc2025.FieldConstants.*;
import static org.littletonrobotics.frc2025.commands.AutoConstants.*;
import static org.littletonrobotics.frc2025.commands.IntakeCommands.angleDifferenceWeight;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.*;
import java.util.stream.Collectors;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.FieldConstants.StagingPositions;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.frc2025.util.MirrorUtil;
import org.littletonrobotics.frc2025.util.VirtualSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoTracker extends VirtualSubsystem {
  private static final LoggedTunableNumber lookaheadSecs =
      new LoggedTunableNumber("Auto/Tracker/LookaheadSecs", 0.15);
  private static final LoggedTunableNumber coralMaxViewDistance =
      new LoggedTunableNumber("Auto/Tracker/CoralMaxViewDistance", 1.2);
  private static final LoggedTunableNumber coralMaxViewAngle =
      new LoggedTunableNumber("Auto/Tracker/CoralMaxViewAngle", 35.0);
  private static final LoggedTunableNumber iceCreamYDistance =
      new LoggedTunableNumber("Auto/Tracker/IceCreamYDistance", 0.5);
  private static final LoggedTunableNumber iceCreamFrontXDistance =
      new LoggedTunableNumber("Auto/Tracker/IceCreamFrontXDistance", 0.5);
  private static final LoggedTunableNumber iceCreamBackXDistance =
      new LoggedTunableNumber("Auto/Tracker/IceCreamBackXDistance", 1.5);

  private static final Set<IntakingLocation> iceCreamLocations =
      Set.of(
          IntakingLocation.CLOSE_CREAM, IntakingLocation.MIDDLE_CREAM, IntakingLocation.FAR_CREAM);

  private final LoggedDashboardChooser<StartingBranch> startingBranchChooser =
      new LoggedDashboardChooser<>("Starting Branch?");
  private final LoggedDashboardChooser<EndingBranch> endingBranchChooser =
      new LoggedDashboardChooser<>("Ending Branch?");
  private final List<Pair<IntakingLocation, LoggedDashboardChooser<Boolean>>>
      intakingLocationChoosers = new ArrayList<>(4);
  @AutoLogOutput private StartingBranch startingBranch = null;
  @AutoLogOutput private EndingBranch endingBranch = null;

  private final Set<CoralObjective> openAutoObjectives = new HashSet<>();
  private final Set<IntakingLocation> validIntakingLocations = new HashSet<>();
  private final Set<IntakingLocation> openIntakingLocations = new HashSet<>();
  @Getter private Optional<Translation2d> nearestValidCoral = Optional.empty();
  private boolean needsToReset = false;

  @AutoLogOutput private Pose2d predictedRobot;

  protected AutoTracker() {
    // Initialize choosers
    for (var startingBranch : StartingBranch.values()) {
      if (startingBranch == StartingBranch.SUPER) {
        startingBranchChooser.addDefaultOption(startingBranch.name(), startingBranch);
      } else {
        startingBranchChooser.addOption(startingBranch.name(), startingBranch);
      }
    }
    for (var endingBranch : EndingBranch.values()) {
      if (endingBranch == EndingBranch.SAFE) {
        endingBranchChooser.addDefaultOption(endingBranch.name(), endingBranch);
      } else {
        endingBranchChooser.addOption(endingBranch.name(), endingBranch);
      }
    }
    for (var intakingLocation : IntakingLocation.values()) {
      var chooser = new LoggedDashboardChooser<Boolean>("Intake from " + intakingLocation + "?");
      if (intakingLocation == IntakingLocation.STATION) {
        chooser.addDefaultOption("YES", true);
        chooser.addOption("NO", false);
      } else {
        chooser.addOption("YES", true);
        chooser.addDefaultOption("NO", false);
      }
      intakingLocationChoosers.add(Pair.of(intakingLocation, chooser));
    }
  }

  @Override
  public void periodic() {
    if (DriverStation.isTeleopEnabled()) {
      return;
    } else if (!DriverStation.isAutonomousEnabled()
        || startingBranch == null
        || endingBranch == null) {
      // Update and reset open auto locations
      boolean updateValidBranches = false;
      if (startingBranch == null || startingBranchChooser.get() != startingBranch) {
        startingBranch = startingBranchChooser.get();
        updateValidBranches = true;
      }
      if (endingBranch == null || endingBranchChooser.get() != endingBranch) {
        endingBranch = endingBranchChooser.get();
        updateValidBranches = true;
      }

      if (updateValidBranches || needsToReset) {
        openAutoObjectives.clear();
        openAutoObjectives.addAll(getObjectives(startingBranch, endingBranch));
      }

      validIntakingLocations.clear();
      validIntakingLocations.addAll(
          intakingLocationChoosers.stream()
              .filter(pair -> pair.getSecond().get())
              .map(Pair::getFirst)
              .collect(Collectors.toSet()));
      if (openIntakingLocations.isEmpty()
          || !validIntakingLocations.equals(openIntakingLocations)
          || needsToReset) {
        openIntakingLocations.clear();
        openIntakingLocations.addAll(validIntakingLocations);
      }
      needsToReset = false;
    } else {
      needsToReset = true;

      // Update predicted robot member
      Pose2d robot = RobotState.getInstance().getEstimatedPose();
      predictedRobot =
          robot.exp(RobotState.getInstance().getRobotVelocity().toTwist2d(lookaheadSecs.get()));

      // Get valid coral
      Set<Translation2d> coralTranslations =
          RobotState.getInstance().getCoralTranslations().stream()
              .filter(
                  translation -> {
                    var transformedCoral = MirrorUtil.apply(AllianceFlipUtil.apply(translation));
                    return validIntakingLocations.stream()
                        .anyMatch(
                            intakingLocation -> {
                              if (intakingLocation == IntakingLocation.STATION) {
                                return iceCreamLocations.stream()
                                    .noneMatch(
                                        iceCreamLocation ->
                                            isIceCreamCoral(iceCreamLocation, transformedCoral));
                              }
                              return isIceCreamCoral(intakingLocation, transformedCoral);
                            });
                  })
              .collect(Collectors.toSet());

      // Find nearest
      nearestValidCoral =
          coralTranslations.stream()
              .filter(
                  coral ->
                      Constants.getRobot() != Constants.RobotType.SIMBOT
                          || (predictedRobot.getTranslation().getDistance(coral) <= 3.0
                              && Math.abs(
                                      predictedRobot
                                              .getRotation()
                                              .rotateBy(Rotation2d.kPi)
                                              .getDegrees()
                                          - (coral
                                              .minus(predictedRobot.getTranslation())
                                              .getAngle()
                                              .getDegrees()))
                                  <= 60.0))
              .min(
                  Comparator.comparingDouble(
                      coral ->
                          coral.getDistance(predictedRobot.getTranslation())
                              + Math.abs(
                                  coral
                                          .minus(robot.getTranslation())
                                          .getAngle()
                                          .minus(robot.getRotation().plus(Rotation2d.kPi))
                                          .getRadians()
                                      * angleDifferenceWeight.get())));

      // Update ice cream locations
      for (var iceCreamLocation :
          validIntakingLocations.stream()
              .filter(iceCreamLocations::contains)
              .collect(Collectors.toSet())) {
        // Find if looking in direction of ice cream
        Translation2d iceCreamTranslation =
            AllianceFlipUtil.apply(
                StagingPositions.iceCreams[
                    MirrorUtil.shouldMirror()
                        ? 2 - iceCreamLocation.getIceCreamIndex()
                        : iceCreamLocation.getIceCreamIndex()]);
        if (robot.getTranslation().getDistance(iceCreamTranslation) >= coralMaxViewDistance.get()
            || Math.abs(
                    robot
                        .getTranslation()
                        .minus(iceCreamTranslation)
                        .getAngle()
                        .minus(robot.getRotation())
                        .getDegrees())
                >= coralMaxViewAngle.get()) continue;

        // Add or remove if there is a coral there
        if (coralTranslations.stream()
            .anyMatch(
                translation ->
                    isIceCreamCoral(
                        iceCreamLocation, MirrorUtil.apply(AllianceFlipUtil.apply(translation))))) {
          openIntakingLocations.add(iceCreamLocation);
        } else {
          openIntakingLocations.remove(iceCreamLocation);
        }
      }
    }

    Logger.recordOutput(
        "AutoTracker/OpenAutoObjectives", openAutoObjectives.toArray(CoralObjective[]::new));
    Logger.recordOutput(
        "AutoTracker/OpenIntakingLocations",
        openIntakingLocations.toArray(IntakingLocation[]::new));
    Logger.recordOutput(
        "AutoTracker/NearestValidCoral",
        nearestValidCoral
            .map(
                translation ->
                    new Translation3d[] {
                      new Translation3d(translation.getX(), translation.getY(), coralDiameter / 2.0)
                    })
            .orElse(new Translation3d[] {}));

    // Record cycle time
    LoggedTracer.record("AutoTracker");
  }

  public Pose2d getStartingPose() {
    return AllianceFlipUtil.apply(MirrorUtil.apply(startingBranch.startingPose));
  }

  public Optional<ReefLevel> getLevel() {
    return openAutoObjectives.stream()
        .max(Comparator.comparingInt(objective -> objective.reefLevel().levelNumber))
        .map(CoralObjective::reefLevel);
  }

  /** Returns empty optional if no valid objectives left to score on. */
  public Optional<CoralObjective> getCoralObjective(ReefLevel level) {
    return openAutoObjectives.stream()
        .filter(objective -> objective.reefLevel() == level)
        .min(
            Comparator.comparingDouble(
                objective ->
                    predictedRobot
                        .getTranslation()
                        .getDistance(
                            AllianceFlipUtil.apply(
                                    MirrorUtil.apply(
                                        AutoScoreCommands.getCoralScorePose(objective, false)))
                                .getTranslation())))
        .map(MirrorUtil::apply);
  }

  public Optional<IntakingLocation> getIntakeLocation(IntakingLocation previousLocation) {
    return openIntakingLocations.stream()
        .filter(
            intakingLocation -> {
              if (openIntakingLocations.size() == 1) return true;
              if (previousLocation == null) return true;
              return intakingLocation != previousLocation;
            })
        .min(
            Comparator.comparingDouble(
                intakingLocation ->
                    predictedRobot
                        .getTranslation()
                        .getDistance(
                            getIntakePose(predictedRobot, intakingLocation).getTranslation())));
  }

  public void setObjectiveScored(CoralObjective objective) {
    openAutoObjectives.remove(MirrorUtil.apply(objective));
  }

  /** Returns set of coral objectives given starting and ending branches. */
  private static Set<CoralObjective> getObjectives(
      StartingBranch startingBranch, EndingBranch endingBranch) {
    Set<CoralObjective> objectives = new HashSet<>();
    for (int i = startingBranch.branchId;
        i <= (endingBranch.branchId < 11 ? endingBranch.branchId + 12 : endingBranch.branchId);
        i++) {
      int branchId = i % 12;
      objectives.add(new CoralObjective(branchId, ReefLevel.L4));
      if ((branchId / 2) % 2 == 0) {
        objectives.add(new CoralObjective(branchId, ReefLevel.L2));
      }
    }
    return objectives;
  }

  private static boolean isIceCreamCoral(
      IntakingLocation iceCream, Translation2d transformedCoralTranslation) {
    if (iceCream == IntakingLocation.STATION) return false;
    Translation2d error =
        transformedCoralTranslation.minus(StagingPositions.iceCreams[iceCream.iceCreamIndex]);
    return Math.abs(error.getY()) <= iceCreamYDistance.get()
        && error.getX() <= iceCreamFrontXDistance.get()
        && error.getX() >= -iceCreamBackXDistance.get();
  }

  @RequiredArgsConstructor
  public enum StartingBranch {
    SUPER(8, closeCageStart),
    SUPER_ADJACENT(9, closeCageStart),
    FAR(10, farCageStart),
    SUPER_FAR(11, farCageStart);

    private final int branchId;
    private final Pose2d startingPose;
  }

  @RequiredArgsConstructor
  public enum EndingBranch {
    SAFE(11),
    HALFWAY(0),
    HALFWAY_AND_ONE(1),
    UNETHICAL(2);

    private final int branchId;
  }

  @RequiredArgsConstructor
  @Getter
  public enum IntakingLocation {
    STATION(-1),
    CLOSE_CREAM(0),
    MIDDLE_CREAM(1),
    FAR_CREAM(2);

    private final int iceCreamIndex;
  }
}
