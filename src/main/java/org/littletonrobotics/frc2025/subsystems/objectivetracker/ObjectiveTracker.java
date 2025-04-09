// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.objectivetracker;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.stream.Collector;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.AlgaeObjective;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.commands.AutoScoreCommands;
import org.littletonrobotics.frc2025.commands.DriveCommands;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.frc2025.util.VirtualSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class ObjectiveTracker extends VirtualSubsystem {
  private static final String inactiveColor = "#242424";
  private static final String completeColor = "#00ff00";
  private static final String incompleteColor = "#ff0000";
  private static final String skipColor = "#ffff00";
  private static final String simpleStrategy = "4321";

  private static final Set<CoralObjective> l1CoralObjectives =
      IntStream.rangeClosed(0, 11)
          .mapToObj(id -> new CoralObjective(id, ReefLevel.L1))
          .collect(Collectors.toSet());

  private static final LoggedTunableNumber lookaheadS =
      new LoggedTunableNumber("ObjectiveTracker/LookaheadS", 0.35);

  private final ReefControlsIO io;
  private final ReefControlsIOInputsAutoLogged inputs = new ReefControlsIOInputsAutoLogged();

  private ReefState reefState = ReefState.initial;
  private ReefState previousReefState;
  private boolean coopState = false;
  private int selectedLevel = 0;
  private boolean lastCoopState = false;

  private final Set<CoralObjective> availableBranches = new HashSet<>();
  private final Set<CoralObjective> availableUnblockedBranches = new HashSet<>();
  private final Set<CoralObjective> nearbyBranches = new HashSet<>();
  private final Set<CoralObjective> nearbyUnblockedBranches = new HashSet<>();

  private final Set<AlgaeObjective> presentAlgae = new HashSet<>();

  private List<CoralPriority> fullStrategy = new ArrayList<>();
  private final List<CoralPriority> uncompletedPriorities = new ArrayList<>();
  private List<CoralPriority> openPriorities = new ArrayList<>();

  @Getter private Optional<ReefLevel> firstLevel = Optional.empty();
  @Getter private Optional<ReefLevel> secondLevel = Optional.empty();
  @Getter private Optional<AlgaeObjective> algaeObjective;

  private final LoggedNetworkString strategyInput =
      new LoggedNetworkString("/SmartDashboard/Strategy", "747372714321");
  private String previousStrategy = null;
  private final StringPublisher[] strategyNamesPublishers = new StringPublisher[8];
  private final StringPublisher[] strategyCompletedPublishers = new StringPublisher[8];
  private final LoggedDashboardChooser<String> dashboardLevelChooser =
      new LoggedDashboardChooser<>("Reef Level");

  // Override (s)
  private BooleanSupplier forceSimpleCoralStrategy = () -> false;

  @AutoLogOutput(key = "ObjectiveTracker/PredictedPose")
  private Pose2d predictedRobot = Pose2d.kZero;

  private Pose2d robot = Pose2d.kZero;

  private DoubleSupplier driverX = () -> 0.0;
  private DoubleSupplier driverY = () -> 0.0;
  private DoubleSupplier driverOmega = () -> 0.0;
  private BooleanSupplier robotRelative = () -> false;

  public ObjectiveTracker(ReefControlsIO io) {
    this.io = io;

    dashboardLevelChooser.addDefaultOption("   Auto   ", "Auto");
    for (var level : ReefLevel.values()) {
      dashboardLevelChooser.addOption(
          switch (level) {
            case L1 -> "   Level 1   ";
            case L2 -> "   Level 2   ";
            case L3 -> "   Level 3   ";
            case L4 -> "   Level 4   ";
          },
          level.toString());
    }

    var table =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getSubTable("Objective Tracker");
    for (int i = 0; i < 8; i++) {
      strategyNamesPublishers[i] = table.getStringTopic("Priority #" + (i + 1)).publish();
      strategyCompletedPublishers[i] =
          table.getStringTopic("Priority #" + (i + 1) + " Completed").publish();

      strategyNamesPublishers[i].set("");
      strategyCompletedPublishers[i].set(inactiveColor);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ReefControls", inputs);

    // Update strategy when input changed
    boolean strategyChanged = false;
    if (!getStrategyString().equals(previousStrategy)) {
      strategyChanged = true;
      parseStrategy();
    }

    // Update reef state from inputs
    if (inputs.level1State.length > 0) {
      reefState = new ReefState(reefState.coral(), reefState.algae(), inputs.level1State[0]);
    }
    final int[][] inputLevelStates =
        new int[][] {inputs.level2State, inputs.level3State, inputs.level4State};
    for (int i = 0; i < 3; i++) {
      if (inputLevelStates[i].length > 0) {
        boolean[] levelState = new boolean[12];
        for (int j = 0; j < 12; j++) {
          levelState[j] = (inputLevelStates[i][0] & (1 << j)) != 0;
        }
        reefState.coral()[i] = levelState;
      }
    }
    if (inputs.algaeState.length > 0) {
      boolean[] algae = new boolean[6];
      for (int j = 0; j < 6; j++) {
        algae[j] = (inputs.algaeState[0] & (1 << j)) != 0;
      }
      reefState = new ReefState(reefState.coral(), algae, reefState.troughCount());
    }
    if (inputs.coopState.length > 0) {
      coopState = inputs.coopState[0];
    }
    if (DriverStation.getMatchType() == MatchType.Elimination) {
      coopState = false;
    }

    // If state has changed, recalculate
    if (!reefState.equals(previousReefState) || coopState != lastCoopState || strategyChanged) {
      previousReefState = reefState.clone();
      lastCoopState = coopState;

      // Find available branches
      availableBranches.clear();
      availableUnblockedBranches.clear();
      availableBranches.addAll(l1CoralObjectives);
      availableUnblockedBranches.addAll(l1CoralObjectives);
      for (int level = 0; level < 3; level++) {
        for (int branch = 0; branch < 12; branch++) {
          // Check if coral already placed on branchId
          if (reefState.coral()[level][branch]) continue;
          ReefLevel reefLevel = ReefLevel.valueOf("L" + (level + 2));
          availableBranches.add(new CoralObjective(branch, reefLevel));

          if (level != 2) {
            // For L2 and L3 check algae
            // Get associated algae location
            int face = Math.floorDiv(branch, 2);
            if (level == 1 && reefState.algae()[face]) continue;
            else if (face % 2 == 1 && reefState.algae()[face]) continue;
          }
          // Add to available branches
          availableUnblockedBranches.add(new CoralObjective(branch, reefLevel));
        }
      }
      logAvailableBranches(availableBranches, "AvailableBranches");
      logAvailableBranches(availableUnblockedBranches, "AvailableUnblockedBranches");

      presentAlgae.clear();
      for (int i = 0; i < 6; i++) {
        if (reefState.algae[i]) presentAlgae.add(new AlgaeObjective(i));
      }
      if (presentAlgae.isEmpty()) {
        Logger.recordOutput("ObjectiveTracker/ReefState/PresentAlgae", new AlgaeObjective[] {});
      } else {
        Logger.recordOutput(
            "ObjectiveTracker/ReefState/PresentAlgae", presentAlgae.toArray(AlgaeObjective[]::new));
      }

      // Update uncompletedPriorities
      uncompletedPriorities.clear();
      Optional<CoralPriority> uselessPriority = Optional.empty();
      if (coopState) {
        // Decide which 7 coral we should give up on based on completion
        uselessPriority =
            Set.of(CoralPriority._74, CoralPriority._73, CoralPriority._72, CoralPriority._71)
                .stream()
                .filter(coralPriority -> !coralPriority.complete(reefState))
                .sorted(
                    Comparator.comparingInt(
                            priority -> {
                              int index = fullStrategy.indexOf(priority);
                              return index == -1 ? Integer.MAX_VALUE : index;
                            })
                        .reversed())
                .min(
                    Comparator.comparingInt(
                        priority -> {
                          if (priority.getLevel() == ReefLevel.L1) return reefState.troughCount;
                          int count = 0;
                          for (int i = 0; i < 12; i++) {
                            if (reefState.coral[priority.getLevel().ordinal() - 1][i]) count++;
                          }
                          return count;
                        }));
      }

      for (int i = 0; i < fullStrategy.size(); i++) {
        // Show dashboard
        if (uselessPriority.isPresent() && uselessPriority.get() == fullStrategy.get(i)) {
          strategyCompletedPublishers[i].set(skipColor);
        } else if (fullStrategy.get(i).complete(reefState)) {
          strategyCompletedPublishers[i].set(completeColor);
        } else {
          strategyCompletedPublishers[i].set(incompleteColor);
          uncompletedPriorities.add(fullStrategy.get(i));
        }
      }

      // Log reef state
      for (int i = 0; i < reefState.coral.length; i++) {
        for (int j = 0; j < reefState.coral[i].length; j++) {
          Logger.recordOutput(
              "ObjectiveTracker/ReefState/CoralState/Level" + (i + 2) + "/Branch" + (j + 1),
              reefState.coral[i][j]);
        }
      }
      for (int i = 0; i < reefState.algae.length; i++) {
        Logger.recordOutput("ObjectiveTracker/ReefState/AlgaeState/" + (i + 1), reefState.algae[i]);
      }

      // Show state of reef in 3d
      Set<Pose3d> coralPoses = new HashSet<>();
      for (int level = 0; level < reefState.coral().length; level++) {
        for (int j = 0; j < reefState.coral()[0].length; j++) {
          if (!reefState.coral()[level][j]) continue;
          coralPoses.add(
              AllianceFlipUtil.apply(
                  FieldConstants.Reef.branchPositions
                      .get(j)
                      .get(ReefLevel.fromLevel(level + 1))
                      .transformBy(
                          GeomUtil.toTransform3d(
                              new Pose3d(
                                  level == 2
                                      ? new Translation3d(
                                          -Units.inchesToMeters(7.2), 0, Units.inchesToMeters(1.3))
                                      : new Translation3d(
                                          -Units.inchesToMeters(11.875) / 2.5, 0.0, 0.0),
                                  level == 2
                                      ? new Rotation3d(0, Units.degreesToRadians(25), 0)
                                      : Rotation3d.kZero)))));
        }
      }
      Logger.recordOutput("ObjectiveTracker/3DView/Coral", coralPoses.toArray(Pose3d[]::new));

      Set<Translation3d> algaePoses = new HashSet<>();
      for (int i = 0; i < 6; i++) {
        if (!reefState.algae()[i]) continue;
        var firstBranchPose = FieldConstants.Reef.branchPositions.get(i * 2).get(ReefLevel.L2);
        var secondBranchPose = FieldConstants.Reef.branchPositions.get(i * 2 + 1).get(ReefLevel.L3);
        algaePoses.add(
            AllianceFlipUtil.apply(
                firstBranchPose
                    .getTranslation()
                    .interpolate(secondBranchPose.getTranslation(), 0.5)
                    .plus(
                        new Translation3d(
                            -FieldConstants.algaeDiameter / 3.0,
                            new Rotation3d(
                                0.0,
                                -35.0 / 180.0 * Math.PI,
                                firstBranchPose.getRotation().getZ())))
                    .plus(
                        new Translation3d(
                            0.0,
                            0.0,
                            (i % 2 == 0)
                                ? secondBranchPose.getZ() - firstBranchPose.getZ()
                                : 0.0))));
      }
      Logger.recordOutput(
          "ObjectiveTracker/3DView/Algae", algaePoses.toArray(Translation3d[]::new));
    }

    // Publish names
    for (int i = 0; i < fullStrategy.size(); i++) {
      strategyNamesPublishers[i].set(fullStrategy.get(i).getName());
    }
    for (int i = fullStrategy.size(); i < 8; i++) {
      strategyNamesPublishers[i].set("");
    }

    // Publish state to dashboard
    if (inputs.selectedLevel.length > 0) {
      selectedLevel = inputs.selectedLevel[0];
    }
    io.setSelectedLevel(selectedLevel);
    io.setLevel1State(reefState.troughCount());
    final int[] levelStates = new int[] {0, 0, 0};
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 12; j++) {
        if (reefState.coral()[i][j]) {
          levelStates[i] |= 1 << j;
        }
      }
    }
    io.setLevel2State(levelStates[0]);
    io.setLevel3State(levelStates[1]);
    io.setLevel4State(levelStates[2]);
    int algaeState = 0;
    for (int i = 0; i < 6; i++) {
      if (reefState.algae()[i]) {
        algaeState |= 1 << i;
      }
    }
    io.setAlgaeState(algaeState);
    io.setCoopState(coopState);
    io.setElims(DriverStation.getMatchType() == MatchType.Elimination);

    // Calculate predicted robot and robot
    robot = AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose());
    // Calculate wanted speeds
    var linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(driverX.getAsDouble(), driverY.getAsDouble())
            .times(DriveConstants.maxLinearSpeed);
    double omega =
        DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble())
            * DriveConstants.maxAngularSpeed;
    ChassisSpeeds wantedSpeeds =
        new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);
    predictedRobot =
        robot.exp(
            (robotRelative.getAsBoolean()
                    ? wantedSpeeds
                    : ChassisSpeeds.fromFieldRelativeSpeeds(wantedSpeeds, robot.getRotation()))
                .toTwist2d(lookaheadS.get()));

    // Calculate nearby branches that do not have coral on them
    nearbyBranches.clear();
    availableBranches.stream()
        .filter(
            objective ->
                Math.abs(
                        predictedRobot
                            .relativeTo(AutoScoreCommands.getBranchPose(objective))
                            .getTranslation()
                            .getAngle()
                            .getDegrees())
                    <= 100.0)
        .forEach(nearbyBranches::add);

    // Calculate current open priorities in order
    openPriorities.clear();
    int count = 0;
    for (var priority : uncompletedPriorities) {
      if (openPriorities.stream()
              .noneMatch(objective -> priority.getLevel() == objective.getLevel())
          && nearbyBranches.stream()
              .anyMatch(objective -> priority.getLevel() == objective.reefLevel())) {
        openPriorities.add(priority);
        count++;
      }
      if (count == 2) break;
    }
    Logger.recordOutput(
        "ObjectiveTracker/Strategy/OpenPriorities", openPriorities.toArray(CoralPriority[]::new));

    // Get first and second level priorities
    firstLevel = getLevel(false);
    secondLevel = getLevel(true);

    // Calculate the unblocked nearby branches
    nearbyUnblockedBranches.clear();
    nearbyBranches.stream()
        .filter(availableUnblockedBranches::contains)
        .forEach(nearbyUnblockedBranches::add);

    // Show strategy on LEDs
    Leds.getInstance().firstPriorityLevel = firstLevel;
    Leds.getInstance().secondPriorityLevel = secondLevel;
    Leds.getInstance().firstPriorityBlocked =
        firstLevel
            .map(
                reefLevel ->
                    nearbyUnblockedBranches.stream()
                        .noneMatch(objective -> reefLevel == objective.reefLevel()))
            .orElse(true);
    Leds.getInstance().secondPriorityBlocked =
        secondLevel
            .map(
                reefLevel ->
                    nearbyUnblockedBranches.stream()
                        .noneMatch(objective -> reefLevel == objective.reefLevel()))
            .orElse(true);

    // Get nearest algae objective
    algaeObjective =
        presentAlgae.stream()
            .filter(
                objective ->
                    robot.relativeTo(AutoScoreCommands.getReefIntakePose(objective)).getX() <= 0.5)
            .min(
                Comparator.comparingDouble(
                    objective ->
                        robot
                            .getTranslation()
                            .getDistance(
                                AutoScoreCommands.getReefIntakePose(objective).getTranslation())));
    algaeObjective.ifPresentOrElse(
        objective ->
            Logger.recordOutput(
                "ObjectiveTracker/Strategy/AlgaeObjective", new AlgaeObjective[] {objective}),
        () ->
            Logger.recordOutput(
                "ObjectiveTracker/Strategy/AlgaeObjective", new AlgaeObjective[] {}));
    // Log override(s)
    Logger.recordOutput(
        "ObjectiveTracker/ForceSimpleStrategy", forceSimpleCoralStrategy.getAsBoolean());

    // Record cycle time
    LoggedTracer.record("ObjectiveTracker");
  }

  private String getStrategyString() {
    if (forceSimpleCoralStrategy.getAsBoolean()) {
      return simpleStrategy;
    } else {
      return strategyInput.get();
    }
  }

  private Optional<ReefLevel> getLevel(boolean secondPriority) {
    if (!dashboardLevelChooser.get().equals("Auto")) {
      return Optional.of(ReefLevel.valueOf(dashboardLevelChooser.get()));
    }
    if (!secondPriority) {
      return !openPriorities.isEmpty()
          ? Optional.of(openPriorities.get(0).getLevel())
          : Optional.empty();
    } else {
      if (openPriorities.size() < 2) return Optional.empty();
      return Optional.of(openPriorities.get(1).getLevel());
    }
  }

  public Optional<CoralObjective> getCoralObjective(ReefLevel level) {
    return nearbyUnblockedBranches.stream()
        .filter(objective -> objective.reefLevel() == level)
        .min(nearestCoralObjectiveComparator(predictedRobot));
  }

  public Optional<CoralObjective> getSuperCoralObjective(
      AlgaeObjective algaeObjective, ReefLevel level) {
    return nearbyBranches.stream()
        .filter(
            objective ->
                objective.reefLevel() == level
                    && algaeObjective != null
                    && objective.branchId() / 2 == algaeObjective.id())
        .min(nearestCoralObjectiveComparator(predictedRobot));
  }

  private static final Set<Character> allowedCharacters = Set.of('1', '2', '3', '4', '7');

  private void parseStrategy() {
    previousStrategy = getStrategyString();
    fullStrategy.clear();
    String filtered =
        getStrategyString()
            .chars()
            .mapToObj(chr -> (char) chr)
            .filter(allowedCharacters::contains)
            .collect(
                Collector.of(
                    StringBuilder::new,
                    StringBuilder::append,
                    StringBuilder::append,
                    StringBuilder::toString));
    Logger.recordOutput("ObjectiveTracker/Strategy/StrategyInput", filtered);
    // Populate with proper enums
    for (int i = 0; i < filtered.length(); i++) {
      if (filtered.charAt(i) == '7' && i + 1 < filtered.length() && filtered.charAt(i + 1) != '7') {
        fullStrategy.add(CoralPriority.valueOf("_" + filtered.substring(i, i + 2)));
        i++;
      } else if (filtered.charAt(i) != '7') {
        fullStrategy.add(CoralPriority.valueOf("_" + filtered.charAt(i)));
      }
    }
    // Remove duplicates
    fullStrategy = new ArrayList<>(fullStrategy.stream().distinct().toList());
    // Add fill priorities
    for (int i = 4; i >= 1; i--) {
      CoralPriority priority = CoralPriority.valueOf("_" + i);
      if (!fullStrategy.contains(priority)) {
        fullStrategy.add(priority);
      }
    }
    Logger.recordOutput(
        "ObjectiveTracker/Strategy/FullStrategy", fullStrategy.toArray(CoralPriority[]::new));

    for (int i = 0; i < fullStrategy.size(); i++) {
      strategyNamesPublishers[i].set(fullStrategy.get(i).getName());
      strategyCompletedPublishers[i].set(inactiveColor);
    }
    for (int i = fullStrategy.size(); i < 8; i++) {
      strategyNamesPublishers[i].set("");
      strategyCompletedPublishers[i].set(inactiveColor);
    }
  }

  public void setOverrides(BooleanSupplier forceSimpleCoralStrategy) {
    this.forceSimpleCoralStrategy = forceSimpleCoralStrategy;
  }

  public void setDriverInput(
      DoubleSupplier driverX,
      DoubleSupplier driverY,
      DoubleSupplier driverOmega,
      BooleanSupplier robotRelative) {
    this.driverX = driverX;
    this.driverY = driverY;
    this.driverOmega = driverOmega;
    this.robotRelative = robotRelative;
  }

  @Getter
  @RequiredArgsConstructor
  /** All strategies have bias towards higher levels! */
  public enum CoralPriority {
    _74(ReefLevel.L4, false, "7/4"),
    _73(ReefLevel.L3, false, "7/3"),
    _72(ReefLevel.L2, false, "7/2"),
    _71(ReefLevel.L1, false, "7/1"),
    _4(ReefLevel.L4, true, "4"),
    _3(ReefLevel.L3, true, "3"),
    _2(ReefLevel.L2, true, "2"),
    _1(ReefLevel.L1, true, "1");

    private final ReefLevel level;
    private final boolean fill;
    private final String name;

    public boolean complete(ReefState reefState) {
      if (fill) {
        if (level == ReefLevel.L1) return false;
        boolean[] levelState = reefState.coral()[level.ordinal() - 1];
        for (boolean b : levelState) {
          if (!b) return false;
        }
        return true;
      }
      if (level == ReefLevel.L1) return reefState.troughCount() >= 7;
      int count = 0;
      boolean[] levelState = reefState.coral()[level.ordinal() - 1];
      for (boolean b : levelState) {
        if (b) count++;
      }
      return count >= 7;
    }
  }

  private static void logAvailableBranches(Set<CoralObjective> availableBranches, String key) {
    var branchesForLevel =
        availableBranches.stream().collect(Collectors.groupingBy(CoralObjective::reefLevel));
    branchesForLevel.forEach(
        (level, objectives) ->
            Logger.recordOutput(
                "ObjectiveTracker/ReefState/" + key + "/Level" + (level.ordinal() + 1),
                objectives.toArray(CoralObjective[]::new)));
    for (var level : ReefLevel.values()) {
      if (branchesForLevel.get(level) != null) continue;
      Logger.recordOutput(
          "ObjectiveTracker/ReefState/" + key + "/Level" + (level.ordinal() + 1),
          new CoralObjective[] {});
    }
  }

  private static Comparator<CoralObjective> nearestCoralObjectiveComparator(Pose2d robot) {
    return Comparator.comparingDouble(
        (CoralObjective coralObjective) ->
            robot
                .getTranslation()
                .getDistance(
                    AutoScoreCommands.getCoralScorePose(coralObjective, false).getTranslation()));
  }

  private record ReefState(boolean[][] coral, boolean[] algae, int troughCount) {
    public static final ReefState initial =
        new ReefState(
            new boolean[][] {
              new boolean[] {
                false, false, false, false, false, false, false, false, false, false, false, false
              },
              new boolean[] {
                false, false, false, false, false, false, false, false, false, false, false, false
              },
              new boolean[] {
                false, false, false, false, false, false, false, false, false, false, false, false
              }
            },
            new boolean[] {true, true, true, true, true, true},
            0);

    @Override
    public boolean equals(Object o) {
      if (!(o instanceof ReefState reefState)) return false;
      return troughCount == reefState.troughCount
          && Arrays.equals(algae, reefState.algae)
          && Arrays.deepEquals(coral, reefState.coral);
    }

    @Override
    protected ReefState clone() {
      boolean[][] copy = new boolean[coral.length][coral[0].length];
      for (int i = 0; i < copy.length; i++) {
        copy[i] = Arrays.copyOf(coral[i], coral[i].length);
      }
      return new ReefState(copy, Arrays.copyOf(algae, algae.length), troughCount);
    }
  }
}
