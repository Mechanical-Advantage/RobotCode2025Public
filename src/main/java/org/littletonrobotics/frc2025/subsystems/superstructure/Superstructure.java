// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import static org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants.*;
import static org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructurePose.algaeSuperPositionDeg;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Builder;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.Constants.Mode;
import org.littletonrobotics.frc2025.Constants.RobotType;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.commands.AutoScoreCommands;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureStateData.Height;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.Dispenser;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.Elevator;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Superstructure extends SubsystemBase {
  private static final Map<SuperstructureState, SuperstructureState> coralEjectPairs =
      Map.of(
          SuperstructureState.L1_CORAL, SuperstructureState.L1_CORAL_EJECT,
          SuperstructureState.L2_CORAL, SuperstructureState.L2_CORAL_EJECT,
          SuperstructureState.L3_CORAL, SuperstructureState.L3_CORAL_EJECT,
          SuperstructureState.L4_CORAL, SuperstructureState.L4_CORAL_EJECT,
          SuperstructureState.ALGAE_L2_CORAL, SuperstructureState.ALGAE_L2_CORAL_EJECT,
          SuperstructureState.ALGAE_L3_CORAL, SuperstructureState.ALGAE_L3_CORAL_EJECT,
          SuperstructureState.ALGAE_L4_CORAL, SuperstructureState.ALGAE_L4_CORAL_EJECT);

  private final Elevator elevator;
  private final Dispenser dispenser;

  private final Graph<SuperstructureState, EdgeCommand> graph =
      new DefaultDirectedGraph<>(EdgeCommand.class);

  private EdgeCommand edgeCommand;

  @Getter private SuperstructureState state = SuperstructureState.START;
  private SuperstructureState next = null;
  @Getter private SuperstructureState goal = SuperstructureState.START;
  private boolean wasDisabled = false;

  @AutoLogOutput(key = "Superstructure/EStopped")
  private boolean isEStopped = false;

  private LoggedNetworkBoolean characterizationModeOn =
      new LoggedNetworkBoolean("/SmartDashboard/Characterization Mode On", false);

  private BooleanSupplier disableOverride = () -> false;
  private final Alert driverDisableAlert =
      new Alert("Superstructure disabled due to driver override.", Alert.AlertType.kWarning);
  private final Alert emergencyDisableAlert =
      new Alert(
          "Superstructure emergency disabled due to high position error. Disable the superstructure manually and reenable to reset.",
          Alert.AlertType.kError);

  private final SuperstructureVisualizer measuredVisualizer =
      new SuperstructureVisualizer("Measured");
  private final SuperstructureVisualizer setpointVisualizer =
      new SuperstructureVisualizer("Setpoint");
  private final SuperstructureVisualizer goalVisualizer = new SuperstructureVisualizer("Goal");

  @Setter private Optional<SuperstructureState> reefDangerState = Optional.empty();

  @AutoLogOutput @Getter private boolean requestFunnelOuttake = false;
  @AutoLogOutput private boolean forceFastConstraints = false;

  public Superstructure(Elevator elevator, Dispenser dispenser) {
    this.elevator = elevator;
    this.dispenser = dispenser;

    // Updating E Stop based on disabled override
    new Trigger(() -> disableOverride.getAsBoolean())
        .onFalse(Commands.runOnce(() -> isEStopped = false).ignoringDisable(true));

    // Add states as vertices
    for (var state : SuperstructureState.values()) {
      graph.addVertex(state);
    }

    // Populate edges
    // Add edge from start to stow
    graph.addEdge(
        SuperstructureState.START,
        SuperstructureState.STOW,
        EdgeCommand.builder()
            .command(
                runSuperstructureExtras(SuperstructureState.STOW)
                    .andThen(
                        runDispenserPivot(
                            () ->
                                Rotation2d.fromDegrees(
                                    MathUtil.clamp(
                                        dispenser.getPivotAngle().getDegrees(),
                                        pivotMinSafeAngleDeg.get(),
                                        pivotMaxSafeAngleDeg.get()))),
                        Commands.parallel(
                                Commands.waitSeconds(0.2).andThen(elevator.homingSequence()))
                            .deadlineFor(
                                Commands.startEnd(
                                    () -> requestFunnelOuttake = true,
                                    () -> requestFunnelOuttake = false)),
                        runSuperstructurePose(SuperstructureState.STOW.getValue().getPose()),
                        Commands.waitUntil(this::mechanismsAtGoal)))
            .build());

    graph.addEdge(
        SuperstructureState.AUTO_START,
        SuperstructureState.STOW,
        EdgeCommand.builder()
            .command(
                Commands.runOnce(elevator::setHome)
                    .onlyIf(() -> Constants.getRobot() != RobotType.SIMBOT)
                    .andThen(
                        runSuperstructurePose(SuperstructurePose.Preset.STOW.getPose()),
                        Commands.waitUntil(this::mechanismsAtGoal),
                        runSuperstructureExtras(SuperstructureState.STOW)))
            .build());

    graph.addEdge(
        SuperstructureState.SAFETY,
        SuperstructureState.STOW,
        EdgeCommand.builder()
            .command(
                runDispenserPivot(
                        () ->
                            Rotation2d.fromDegrees(
                                MathUtil.clamp(
                                    dispenser.getPivotAngle().getDegrees(),
                                    pivotMinSafeAngleDeg.get(),
                                    pivotMaxSafeAngleDeg.get())))
                    .andThen(
                        runSuperstructureExtras(SuperstructureState.STOW),
                        Commands.waitUntil(dispenser::isAtGoal),
                        runSuperstructurePose(SuperstructureState.STOW.getValue().getPose()),
                        Commands.waitUntil(this::mechanismsAtGoal)))
            .build());

    graph.addEdge(
        SuperstructureState.CHARACTERIZATION,
        SuperstructureState.STOW,
        EdgeCommand.builder()
            .command(Commands.idle(this).until(() -> !characterizationModeOn.get()))
            .build());

    final Set<SuperstructureState> freeNoAlgaeStates =
        Set.of(
            SuperstructureState.STOW,
            SuperstructureState.GOODBYE_CORAL,
            SuperstructureState.L1_CORAL,
            SuperstructureState.L2_CORAL,
            SuperstructureState.L3_CORAL,
            SuperstructureState.L4_CORAL,
            SuperstructureState.ALGAE_STOW_INTAKE,
            SuperstructureState.ALGAE_L2_INTAKE,
            SuperstructureState.ALGAE_L3_INTAKE,
            SuperstructureState.ALGAE_ICE_CREAM_INTAKE);

    final Set<SuperstructureState> freeAlgaeStates =
        Set.of(
            SuperstructureState.ALGAE_STOW,
            SuperstructureState.ALGAE_GOODBYE_CORAL,
            SuperstructureState.ALGAE_L2_CORAL,
            SuperstructureState.ALGAE_L3_CORAL,
            SuperstructureState.ALGAE_L4_CORAL,
            SuperstructureState.ALGAE_STOW_INTAKE,
            SuperstructureState.ALGAE_L2_INTAKE,
            SuperstructureState.ALGAE_L3_INTAKE,
            SuperstructureState.ALGAE_ICE_CREAM_INTAKE,
            SuperstructureState.PRE_THROW);

    final Set<SuperstructureState> algaeIntakeStates =
        Set.of(
            SuperstructureState.ALGAE_STOW_INTAKE,
            SuperstructureState.ALGAE_L2_INTAKE,
            SuperstructureState.ALGAE_L3_INTAKE,
            SuperstructureState.ALGAE_ICE_CREAM_INTAKE);

    // Add all free edges
    for (var from : freeNoAlgaeStates) {
      for (var to : freeNoAlgaeStates) {
        if (from == to) continue;
        if (algaeIntakeStates.contains(from) && algaeIntakeStates.contains(to)) continue;
        if (algaeIntakeStates.contains(from)) {
          addEdge(from, to, AlgaeEdge.NO_ALGAE);
        } else {
          addEdge(from, to);
        }
      }
    }

    for (var from : freeAlgaeStates) {
      for (var to : freeAlgaeStates) {
        if (from == to) continue;
        if (algaeIntakeStates.contains(from) && algaeIntakeStates.contains(to)) continue;
        if (algaeIntakeStates.contains(from)) {
          addEdge(from, to, AlgaeEdge.ALGAE);
        } else {
          addEdge(from, to);
        }
      }
    }

    for (var from : algaeIntakeStates) {
      for (var to : algaeIntakeStates) {
        if (from == to) continue;
        addEdge(from, to);
      }
    }

    // Add edges for paired states
    final Set<Pair<SuperstructureState, SuperstructureState>> pairedStates =
        Set.of(
            Pair.of(SuperstructureState.STOW, SuperstructureState.CORAL_INTAKE),
            Pair.of(SuperstructureState.ALGAE_STOW, SuperstructureState.ALGAE_CORAL_INTAKE),
            Pair.of(SuperstructureState.GOODBYE_CORAL, SuperstructureState.GOODBYE_CORAL_EJECT),
            Pair.of(SuperstructureState.L1_CORAL, SuperstructureState.L1_CORAL_EJECT),
            Pair.of(SuperstructureState.L2_CORAL, SuperstructureState.L2_CORAL_EJECT),
            Pair.of(SuperstructureState.L3_CORAL, SuperstructureState.L3_CORAL_EJECT),
            Pair.of(SuperstructureState.L4_CORAL, SuperstructureState.L4_CORAL_EJECT),
            Pair.of(
                SuperstructureState.ALGAE_GOODBYE_CORAL,
                SuperstructureState.ALGAE_GOODBYE_CORAL_EJECT),
            Pair.of(SuperstructureState.ALGAE_L2_CORAL, SuperstructureState.ALGAE_L2_CORAL_EJECT),
            Pair.of(SuperstructureState.ALGAE_L3_CORAL, SuperstructureState.ALGAE_L3_CORAL_EJECT),
            Pair.of(SuperstructureState.ALGAE_L4_CORAL, SuperstructureState.ALGAE_L4_CORAL_EJECT),
            Pair.of(SuperstructureState.PRE_PROCESS, SuperstructureState.PROCESS),
            Pair.of(SuperstructureState.PRE_THROW, SuperstructureState.THROW),
            Pair.of(SuperstructureState.ALGAE_STOW, SuperstructureState.TOSS));
    for (var pair : pairedStates) {
      addEdge(pair.getFirst(), pair.getSecond(), true, AlgaeEdge.NONE, false);
    }

    // Add recoverable algae states
    for (var from :
        Set.of(
            SuperstructureState.ALGAE_STOW,
            SuperstructureState.ALGAE_GOODBYE_CORAL,
            SuperstructureState.ALGAE_L2_CORAL,
            SuperstructureState.ALGAE_L3_CORAL,
            SuperstructureState.ALGAE_L4_CORAL,
            SuperstructureState.PRE_PROCESS,
            SuperstructureState.PRE_THROW,
            SuperstructureState.PROCESS,
            SuperstructureState.TOSS,
            SuperstructureState.THROW)) {
      for (var to : freeNoAlgaeStates) {
        addEdge(from, to, AlgaeEdge.NO_ALGAE);
      }
    }

    // Add miscellaneous edges
    addEdge(
        SuperstructureState.STOW, SuperstructureState.ALGAE_STOW, false, AlgaeEdge.ALGAE, false);
    addEdge(
        SuperstructureState.ALGAE_STOW, SuperstructureState.STOW, false, AlgaeEdge.NO_ALGAE, false);
    addEdge(
        SuperstructureState.ALGAE_STOW,
        SuperstructureState.PRE_PROCESS,
        true,
        AlgaeEdge.NONE,
        false);

    setDefaultCommand(
        runGoal(
            () -> {
              final Pose2d robot = RobotState.getInstance().getEstimatedPose();
              final Pose2d flippedRobot = AllianceFlipUtil.apply(robot);

              // Check danger state
              if (reefDangerState.isPresent()
                  && AutoScoreCommands.withinDistanceToReef(
                      robot,
                      hasAlgae()
                          ? AutoScoreCommands.minDistanceReefClearAlgaeL4.get()
                          : AutoScoreCommands.minDistanceReefClearL4.get())
                  && Math.abs(
                          FieldConstants.Reef.center
                              .minus(flippedRobot.getTranslation())
                              .getAngle()
                              .minus(flippedRobot.getRotation())
                              .getDegrees())
                      <= AutoScoreCommands.minAngleReefClear.get()) {
                // Reset reef danger state when new goal requested
                if (goal != reefDangerState.get()
                    && !(coralEjectPairs.containsKey(reefDangerState.get())
                        && goal == coralEjectPairs.get(reefDangerState.get()))) {
                  reefDangerState = Optional.empty();
                } else {
                  return reefDangerState.get();
                }
              } else if (reefDangerState.isPresent()) {
                reefDangerState = Optional.empty();
              }

              return dispenser.hasAlgae()
                  ? SuperstructureState.ALGAE_STOW
                  : SuperstructureState.STOW;
            }));
  }

  @Override
  public void periodic() {
    // Run periodic
    elevator.periodic();
    dispenser.periodic();

    if (characterizationModeOn.get()) {
      state = SuperstructureState.CHARACTERIZATION;
      next = null;
      Leds.getInstance().characterizationMode = true;
    } else {
      Leds.getInstance().characterizationMode = false;
    }

    if ((DriverStation.isDisabled() || isEStopped) && !wasDisabled && elevator.isHomed()) {
      state = SuperstructureState.SAFETY;
    }
    wasDisabled = DriverStation.isDisabled() || isEStopped;

    if (DriverStation.isDisabled()) {
      next = null;
    } else if (edgeCommand == null || !edgeCommand.getCommand().isScheduled()) {
      // Update edge to new state
      if (next != null) {
        state = next;
        next = null;
      }

      // Schedule next command in sequence
      if (state != goal) {
        bfs(state, goal)
            .ifPresent(
                next -> {
                  this.next = next;
                  edgeCommand = graph.getEdge(state, next);
                  edgeCommand.getCommand().schedule();
                });
      }
    }

    // Tell elevator we are stowed
    elevator.setStowed(
        (state == SuperstructureState.STOW && goal == SuperstructureState.STOW)
            || (state == SuperstructureState.ALGAE_STOW && goal == SuperstructureState.ALGAE_STOW));

    // Tell elevator if we have algae
    elevator.setHasAlgae(dispenser.hasAlgae());

    // Tell dispenser if intaking
    dispenser.setIntaking(
        state == SuperstructureState.CORAL_INTAKE
            || next == SuperstructureState.CORAL_INTAKE
            || state == SuperstructureState.ALGAE_CORAL_INTAKE
            || next == SuperstructureState.ALGAE_CORAL_INTAKE);

    // Force fast constraints
    elevator.setForceFastConstraints(forceFastConstraints);
    dispenser.setForceFastConstraints(forceFastConstraints);

    // E Stop Dispenser and Elevator if Necessary
    isEStopped =
        (isEStopped || elevator.isShouldEStop() || dispenser.isShouldEStop())
            && Constants.getMode() == Mode.REAL;
    elevator.setEStopped(isEStopped);
    dispenser.setEStopped(isEStopped);

    driverDisableAlert.set(disableOverride.getAsBoolean());
    emergencyDisableAlert.set(isEStopped);
    Leds.getInstance().superstructureEstopped = isEStopped;

    // Log state
    Logger.recordOutput("Superstructure/State", state);
    Logger.recordOutput("Superstructure/Next", next);
    Logger.recordOutput("Superstructure/Goal", goal);
    if (edgeCommand != null) {
      Logger.recordOutput(
          "Superstructure/EdgeCommand",
          graph.getEdgeSource(edgeCommand) + " --> " + graph.getEdgeTarget(edgeCommand));
    } else {
      Logger.recordOutput("Superstructure/EdgeCommand", "");
    }
    Logger.recordOutput(
        "Superstructure/ReefDangerState",
        reefDangerState.map(SuperstructureState::toString).orElse(""));

    // Update visualizer
    measuredVisualizer.update(
        elevator.getPositionMeters(),
        dispenser.getPivotAngle(),
        dispenser.hasAlgae(),
        dispenser.hasCoral());
    setpointVisualizer.update(
        elevator.getSetpoint().position,
        Rotation2d.fromRadians(dispenser.getSetpoint().position),
        dispenser.hasAlgae(),
        dispenser.hasCoral());
    goalVisualizer.update(
        elevator.getGoalMeters(),
        Rotation2d.fromRadians(dispenser.getGoal()),
        dispenser.hasAlgae(),
        dispenser.hasCoral());

    // Record cycle time
    LoggedTracer.record("Superstructure");
  }

  public void setOverrides(BooleanSupplier disableOverride) {
    this.disableOverride = disableOverride;
  }

  @AutoLogOutput(key = "Superstructure/AtGoal")
  public boolean atGoal() {
    return state == goal;
  }

  public boolean hasAlgae() {
    return dispenser.hasAlgae();
  }

  public boolean hasCoral() {
    return dispenser.hasCoral();
  }

  public boolean readyForL4() {
    return elevator.getPositionMeters() >= elevatorL4ClearHeight.get();
  }

  public void resetHasCoral() {
    dispenser.resetHasCoral(false);
  }

  public void resetHasAlgae() {
    dispenser.resetHasAlgae(false);
  }

  private void setGoal(SuperstructureState goal) {
    // Don't do anything if goal is the same
    if (this.goal == goal) return;
    this.goal = goal;

    if (next == null) return;

    var edgeToCurrentState = graph.getEdge(next, state);
    // Figure out if we should schedule a different command to get to goal faster
    if (edgeCommand.getCommand().isScheduled()
        && edgeToCurrentState != null
        && isEdgeAllowed(edgeToCurrentState, goal)) {
      // Figure out where we would have gone from the previous state
      bfs(state, goal)
          .ifPresent(
              newNext -> {
                if (newNext == next) {
                  // We are already on track
                  return;
                }

                if (newNext != state && graph.getEdge(next, newNext) != null) {
                  // We can skip directly to the newNext edge
                  edgeCommand.getCommand().cancel();
                  edgeCommand = graph.getEdge(state, newNext);
                  edgeCommand.getCommand().schedule();
                  next = newNext;
                } else {
                  // Follow the reverse edge from next back to the current edge
                  edgeCommand.getCommand().cancel();
                  edgeCommand = graph.getEdge(next, state);
                  edgeCommand.getCommand().schedule();
                  var temp = state;
                  state = next;
                  next = temp;
                }
              });
    }
  }

  public Command runGoal(SuperstructureState goal) {
    return runOnce(() -> setGoal(goal)).andThen(Commands.idle(this));
  }

  public Command runGoal(Supplier<SuperstructureState> goal) {
    return run(() -> setGoal(goal.get()));
  }

  public Command forceEjectDispenser() {
    return startEnd(
        () -> dispenser.setForceEjectForward(true), () -> dispenser.setForceEjectForward(false));
  }

  public Command forceEjectBackward() {
    return startEnd(
        () -> dispenser.setForceEjectReverse(true), () -> dispenser.setForceEjectReverse(false));
  }

  public void setAutoStart() {
    state = SuperstructureState.AUTO_START;
    next = null;
    if (edgeCommand != null) {
      edgeCommand.getCommand().cancel();
    }
    dispenser.resetHasCoral(true);
  }

  private Optional<SuperstructureState> bfs(SuperstructureState start, SuperstructureState goal) {
    // Map to track the parent of each visited node
    Map<SuperstructureState, SuperstructureState> parents = new HashMap<>();
    Queue<SuperstructureState> queue = new LinkedList<>();
    queue.add(start);
    parents.put(start, null); // Mark the start node as visited with no parent
    // Perform BFS
    while (!queue.isEmpty()) {
      SuperstructureState current = queue.poll();
      // Check if we've reached the goal
      if (current == goal) {
        break;
      }
      // Process valid neighbors
      for (EdgeCommand edge :
          graph.outgoingEdgesOf(current).stream()
              .filter(edge -> isEdgeAllowed(edge, goal))
              .toList()) {
        SuperstructureState neighbor = graph.getEdgeTarget(edge);
        // Only process unvisited neighbors
        if (!parents.containsKey(neighbor)) {
          parents.put(neighbor, current);
          queue.add(neighbor);
        }
      }
    }

    // Reconstruct the path to the goal if found
    if (!parents.containsKey(goal)) {
      return Optional.empty(); // Goal not reachable
    }

    // Trace back the path from goal to start
    SuperstructureState nextState = goal;
    while (!nextState.equals(start)) {
      SuperstructureState parent = parents.get(nextState);
      if (parent == null) {
        return Optional.empty(); // No valid path found
      } else if (parent.equals(start)) {
        // Return the edge from start to the next node
        return Optional.of(nextState);
      }
      nextState = parent;
    }
    return Optional.of(nextState);
  }

  private void addEdge(SuperstructureState from, SuperstructureState to) {
    addEdge(from, to, AlgaeEdge.NONE);
  }

  private void addEdge(SuperstructureState from, SuperstructureState to, AlgaeEdge algaeEdge) {
    addEdge(from, to, false, algaeEdge, false);
  }

  private void addEdge(
      SuperstructureState from,
      SuperstructureState to,
      boolean reverse,
      AlgaeEdge algaeEdge,
      boolean restricted) {
    graph.addEdge(
        from,
        to,
        EdgeCommand.builder()
            .command(getEdgeCommand(from, to))
            .algaeEdgeType(algaeEdge)
            .restricted(restricted)
            .build());
    if (reverse) {
      graph.addEdge(
          to,
          from,
          EdgeCommand.builder()
              .command(getEdgeCommand(to, from))
              .algaeEdgeType(algaeEdge)
              .restricted(restricted)
              .build());
    }
  }

  private static final Map<SuperstructureState, SuperstructureState> algaeScoreStates =
      Map.of(
          SuperstructureState.ALGAE_L2_CORAL, SuperstructureState.ALGAE_L2_CORAL_EJECT,
          SuperstructureState.ALGAE_L3_CORAL, SuperstructureState.ALGAE_L3_CORAL_EJECT,
          SuperstructureState.ALGAE_L4_CORAL, SuperstructureState.ALGAE_L4_CORAL_EJECT);

  private Command getEdgeCommand(SuperstructureState from, SuperstructureState to) {
    if (from == SuperstructureState.ALGAE_STOW && to == SuperstructureState.PRE_PROCESS) {
      return runElevator(to.getValue().getPose().elevatorHeight())
          .andThen(
              Commands.waitUntil(elevator::isAtGoal),
              runSuperstructurePose(to.getValue().getPose()),
              Commands.waitUntil(this::mechanismsAtGoal));
    } else if (from == SuperstructureState.PRE_PROCESS && to == SuperstructureState.ALGAE_STOW) {
      return runDispenserPivot(to.getValue().getPose().pivotAngle())
          .andThen(
              Commands.waitUntil(dispenser::isAtGoal),
              runSuperstructurePose(to.getValue().getPose()),
              Commands.waitUntil(this::mechanismsAtGoal));
    }

    if ((algaeScoreStates.containsKey(to) && algaeScoreStates.get(to) != from)
        || (algaeScoreStates.containsKey(from) && algaeScoreStates.get(from) != to)) {
      return runDispenserPivot(() -> Rotation2d.fromDegrees(algaeSuperPositionDeg.get()))
          .andThen(
              Commands.waitUntil(dispenser::isAtGoal),
              runElevator(to.getValue().getPose().elevatorHeight()),
              Commands.waitUntil(elevator::isAtGoal),
              runSuperstructurePose(to.getValue().getPose()),
              Commands.waitUntil(this::mechanismsAtGoal));
    }

    boolean passesThroughCrossMember =
        from.getValue().getHeight().equals(Height.BOTTOM)
            != to.getValue().getHeight().equals(Height.BOTTOM);
    if (passesThroughCrossMember) {
      boolean isGoingDown = to.getValue().getHeight().lowerThan(from.getValue().getHeight());
      SuperstructurePose safeSuperstructureSetpoint =
          new SuperstructurePose(
              () -> {
                // Attempt to reach nearest safe point on elevator
                var currentAngle = from.getValue().getPose().pivotAngle().get();
                if (currentAngle.getDegrees() <= pivotMaxSafeAngleDeg.get()
                    && currentAngle.getDegrees() >= pivotMinSafeAngleDeg.get()) {
                  return to.getValue().getPose().elevatorHeight().getAsDouble();
                }
                return isGoingDown
                    ? from.getValue().getHeight().getPosition() + 0.1
                    : to.getValue().getHeight().getPosition() - 0.1;
              },
              () ->
                  Rotation2d.fromDegrees(
                      MathUtil.clamp(
                          to.getValue().getPose().pivotAngle().get().getDegrees(),
                          pivotMinSafeAngleDeg.get(),
                          pivotMaxSafeAngleDeg.get())));
      return runSuperstructurePose(safeSuperstructureSetpoint)
          .andThen(
              Commands.waitUntil(dispenser::isAtGoal),
              runElevator(to.getValue().getPose().elevatorHeight()),
              Commands.waitUntil(
                  () ->
                      (isGoingDown
                              ? elevator.getPositionMeters()
                                  <= to.getValue().getHeight().getPosition()
                              : elevator.getPositionMeters()
                                  >= to.getValue().getHeight().getPosition())
                          || elevator.isAtGoal()),
              runSuperstructurePose(to.getValue().getPose()),
              Commands.waitUntil(this::mechanismsAtGoal))
          .deadlineFor(runSuperstructureExtras(to));
    } else {
      return runSuperstructurePose(to.getValue().getPose())
          .andThen(Commands.waitUntil(this::mechanismsAtGoal))
          .deadlineFor(runSuperstructureExtras(to));
    }
  }

  public Command runHomingSequence() {
    return runOnce(
        () -> {
          state = SuperstructureState.START;
          next = null;
          if (edgeCommand != null) {
            edgeCommand.command.cancel();
          }
        });
  }

  public Command setCharacterizationMode() {
    return runOnce(
        () -> {
          state = SuperstructureState.CHARACTERIZATION;
          characterizationModeOn.set(true);
          next = null;
          if (edgeCommand != null) {
            edgeCommand.getCommand().cancel();
          }
        });
  }

  private Command runElevator(DoubleSupplier elevatorHeight) {
    return Commands.runOnce(() -> elevator.setGoal(elevatorHeight));
  }

  private Command runDispenserPivot(Supplier<Rotation2d> pivotAngle) {
    return Commands.runOnce(() -> dispenser.setGoal(pivotAngle));
  }

  /** Runs elevator and pivot to {@link SuperstructurePose} pose. Ends immediately. */
  private Command runSuperstructurePose(SuperstructurePose pose) {
    return runElevator(pose.elevatorHeight()).alongWith(runDispenserPivot(pose.pivotAngle()));
  }

  /** Runs dispenser and slam based on {@link SuperstructureState} state. Ends immediately. */
  private Command runSuperstructureExtras(SuperstructureState state) {
    return Commands.runOnce(
        () -> {
          dispenser.setTunnelVolts(state.getValue().getTunnelVolts().getAsDouble());
          dispenser.setGripperGoal(state.getValue().getGripperGoal());
        });
  }

  private boolean isEdgeAllowed(EdgeCommand edge, SuperstructureState goal) {
    return (!edge.isRestricted() || goal == graph.getEdgeTarget(edge))
        && (edge.getAlgaeEdgeType() == AlgaeEdge.NONE
            || dispenser.hasAlgae() == (edge.getAlgaeEdgeType() == AlgaeEdge.ALGAE));
  }

  private boolean mechanismsAtGoal() {
    return elevator.isAtGoal()
        && (dispenser.isAtGoal() || Constants.getRobot() == RobotType.DEVBOT);
  }

  /** Get coral scoring state for level and algae state */
  public static SuperstructureState getScoringState(
      FieldConstants.ReefLevel height, boolean algae, boolean eject) {
    var stateGroup = CoralScoreStateGroup.valueOf(height.toString());
    return algae
        ? eject ? stateGroup.getAlgaeEjectState() : stateGroup.getAlgaeHoldState()
        : eject ? stateGroup.getEjectState() : stateGroup.getHoldState();
  }

  @RequiredArgsConstructor
  @Getter
  private enum CoralScoreStateGroup {
    L1(
        SuperstructureState.L1_CORAL,
        SuperstructureState.L1_CORAL_EJECT,
        SuperstructureState.L1_CORAL,
        SuperstructureState.L1_CORAL_EJECT),
    L2(
        SuperstructureState.L2_CORAL,
        SuperstructureState.L2_CORAL_EJECT,
        SuperstructureState.ALGAE_L2_CORAL,
        SuperstructureState.ALGAE_L2_CORAL_EJECT),
    L3(
        SuperstructureState.L3_CORAL,
        SuperstructureState.L3_CORAL_EJECT,
        SuperstructureState.ALGAE_L3_CORAL,
        SuperstructureState.ALGAE_L3_CORAL_EJECT),
    L4(
        SuperstructureState.L4_CORAL,
        SuperstructureState.L4_CORAL_EJECT,
        SuperstructureState.ALGAE_L4_CORAL,
        SuperstructureState.ALGAE_L4_CORAL_EJECT);

    private final SuperstructureState holdState;
    private final SuperstructureState ejectState;
    private final SuperstructureState algaeHoldState;
    private final SuperstructureState algaeEjectState;
  }

  /** All edge commands should finish and exit properly. */
  @Builder(toBuilder = true)
  @Getter
  public static class EdgeCommand extends DefaultEdge {
    private final Command command;
    @Builder.Default private final boolean restricted = false;
    @Builder.Default private final AlgaeEdge algaeEdgeType = AlgaeEdge.NONE;
  }

  private enum AlgaeEdge {
    NONE,
    NO_ALGAE,
    ALGAE
  }
}
