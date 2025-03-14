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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Builder;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.Constants.Mode;
import org.littletonrobotics.frc2025.Constants.RobotType;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.commands.AlgaeScoreCommands;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.subsystems.superstructure.chariot.Chariot.Goal;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.Dispenser;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.Elevator;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;
  private final Dispenser dispenser;

  private final Graph<SuperstructureState, EdgeCommand> graph =
      new DefaultDirectedGraph<>(EdgeCommand.class);

  private EdgeCommand edgeCommand;

  @Getter private SuperstructureState state = SuperstructureState.START;
  private SuperstructureState next = null;
  private SuperstructureState lastState = SuperstructureState.START;
  private SuperstructureState sourceState = SuperstructureState.START;
  @Getter private SuperstructureState goal = SuperstructureState.START;
  private boolean hasHomedDispenser = false;
  private final Command homeDispenser;

  @AutoLogOutput(key = "Superstructure/EStopped")
  private boolean isEStopped = false;

  private LoggedNetworkBoolean characterizationModeOn =
      new LoggedNetworkBoolean("/SmartDashboard/Characterization Mode On", false);

  private BooleanSupplier disableOverride = () -> false;
  private BooleanSupplier autoCoralStationIntakeOverride = () -> false;
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

  @AutoLogOutput @Getter private boolean requestFunnelIntake = false;
  @AutoLogOutput @Getter private boolean requestFunnelOuttake = false;
  @AutoLogOutput private boolean forceFastConstraints = false;

  public Superstructure(Elevator elevator, Dispenser dispenser) {
    this.elevator = elevator;
    this.dispenser = dispenser;
    homeDispenser = dispenser.homingSequence();

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
                        Commands.parallel(
                                dispenser.homingSequence(),
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
                    .andThen(
                        runSuperstructurePose(SuperstructurePose.Preset.STOW.getPose()),
                        Commands.waitUntil(this::mechanismsAtGoal),
                        runSuperstructureExtras(SuperstructureState.STOW)))
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
            SuperstructureState.ALGAE_L3_INTAKE);

    final Set<SuperstructureState> freeAlgaeStates =
        Set.of(
            SuperstructureState.ALGAE_STOW,
            SuperstructureState.ALGAE_STOW_INTAKE,
            SuperstructureState.ALGAE_L2_INTAKE,
            SuperstructureState.ALGAE_L3_INTAKE,
            SuperstructureState.PRE_TOSS,
            SuperstructureState.PRE_THROWN);

    final Set<SuperstructureState> algaeIntakeStates =
        Set.of(
            SuperstructureState.ALGAE_STOW_INTAKE,
            SuperstructureState.ALGAE_L2_INTAKE,
            SuperstructureState.ALGAE_L3_INTAKE);

    // Add all free edges
    for (var from : freeNoAlgaeStates) {
      for (var to : freeNoAlgaeStates) {
        if (from == to) continue;
        if (algaeIntakeStates.contains(from) && algaeIntakeStates.contains(to)) continue;
        if (algaeIntakeStates.contains(from)) {
          graph.addEdge(
              from,
              to,
              getEdgeCommand(from, to).toBuilder().algaeEdgeType(AlgaeEdge.NO_ALGAE).build());
        } else {
          graph.addEdge(from, to, getEdgeCommand(from, to));
        }
      }
    }

    for (var from : freeAlgaeStates) {
      for (var to : freeAlgaeStates) {
        if (from == to) continue;
        if (algaeIntakeStates.contains(from) && algaeIntakeStates.contains(to)) continue;
        if (algaeIntakeStates.contains(from)) {
          graph.addEdge(
              from,
              to,
              getEdgeCommand(from, to).toBuilder().algaeEdgeType(AlgaeEdge.ALGAE).build());
        } else {
          graph.addEdge(from, to, getEdgeCommand(from, to));
        }
      }
    }

    for (var from : algaeIntakeStates) {
      for (var to : algaeIntakeStates) {
        if (from == to) continue;
        graph.addEdge(from, to, getEdgeCommand(from, to));
      }
    }

    QuintConsumer<SuperstructureState, SuperstructureState, Boolean, AlgaeEdge, Boolean> addEdge =
        (from, to, restricted, algaeEdgeType, reverse) -> {
          graph.addEdge(
              from,
              to,
              getEdgeCommand(from, to).toBuilder()
                  .restricted(restricted)
                  .algaeEdgeType(algaeEdgeType)
                  .build());
          if (reverse) {
            graph.addEdge(
                to,
                from,
                getEdgeCommand(to, from).toBuilder()
                    .restricted(restricted)
                    .algaeEdgeType(algaeEdgeType)
                    .build());
          }
        };

    // Add edges for paired states
    final Set<Pair<SuperstructureState, SuperstructureState>> pairedStates =
        Set.of(
            Pair.of(SuperstructureState.STOW, SuperstructureState.INTAKE),
            Pair.of(SuperstructureState.GOODBYE_CORAL, SuperstructureState.GOODBYE_CORAL_EJECT),
            Pair.of(SuperstructureState.L1_CORAL, SuperstructureState.L1_CORAL_EJECT),
            Pair.of(SuperstructureState.L2_CORAL, SuperstructureState.L2_CORAL_EJECT),
            Pair.of(SuperstructureState.L3_CORAL, SuperstructureState.L3_CORAL_EJECT),
            Pair.of(SuperstructureState.L4_CORAL, SuperstructureState.L4_CORAL_EJECT),
            Pair.of(SuperstructureState.PRE_TOSS, SuperstructureState.TOSS));
    for (var pair : pairedStates) {
      addEdge.accept(pair.getFirst(), pair.getSecond(), false, AlgaeEdge.NONE, true);
    }

    // Add recoverable algae states
    for (var from :
        Set.of(
            SuperstructureState.ALGAE_STOW,
            SuperstructureState.PROCESSED,
            SuperstructureState.PRE_THROWN,
            SuperstructureState.PRE_TOSS,
            SuperstructureState.TOSS,
            SuperstructureState.THROWN)) {
      for (var to : freeNoAlgaeStates) {
        graph.addEdge(
            from,
            to,
            getEdgeCommand(from, to).toBuilder().algaeEdgeType(AlgaeEdge.NO_ALGAE).build());
      }
    }

    // Add miscellaneous edges
    addEdge.accept(
        SuperstructureState.ALGAE_STOW, SuperstructureState.PROCESSED, true, AlgaeEdge.NONE, false);
    addEdge.accept(
        SuperstructureState.PROCESSED,
        SuperstructureState.ALGAE_STOW,
        false,
        AlgaeEdge.ALGAE,
        false);
    addEdge.accept(
        SuperstructureState.THROWN, SuperstructureState.ALGAE_STOW, false, AlgaeEdge.ALGAE, false);
    addEdge.accept(
        SuperstructureState.PRE_TOSS,
        SuperstructureState.ALGAE_STOW,
        false,
        AlgaeEdge.ALGAE,
        false);
    addEdge.accept(
        SuperstructureState.STOW, SuperstructureState.ALGAE_STOW, false, AlgaeEdge.ALGAE, false);
    addEdge.accept(
        SuperstructureState.ALGAE_STOW, SuperstructureState.STOW, false, AlgaeEdge.NO_ALGAE, false);
    addEdge.accept(
        SuperstructureState.PRE_THROWN, SuperstructureState.THROWN, true, AlgaeEdge.NONE, false);
    addEdge.accept(
        SuperstructureState.THROWN, SuperstructureState.PRE_THROWN, false, AlgaeEdge.ALGAE, false);

    setDefaultCommand(
        runGoal(
                () -> {
                  // Check if should intake
                  Pose2d robot =
                      AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose());
                  if (!dispenser.hasCoral()
                      && !dispenser.hasAlgae()
                      && robot.getX() < FieldConstants.fieldLength / 5.0
                      && (robot.getY() < FieldConstants.fieldWidth / 5.0
                          || robot.getY() > FieldConstants.fieldWidth * 4.0 / 5.0)
                      && !autoCoralStationIntakeOverride.getAsBoolean()) {
                    if (state == SuperstructureState.INTAKE) {
                      requestFunnelIntake = true;
                    }
                    return SuperstructureState.INTAKE;
                  }
                  requestFunnelIntake = false;
                  return dispenser.hasAlgae()
                      ? SuperstructureState.ALGAE_STOW
                      : SuperstructureState.STOW;
                })
            .finallyDo(() -> requestFunnelIntake = false));
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

      // Update last state
      if (state != lastState) {
        sourceState = lastState;
        lastState = state;
      }

      // Auto home dispenser when stowing
      if (state == SuperstructureState.STOW
          && goal == SuperstructureState.STOW
          && sourceState != SuperstructureState.INTAKE
          && !dispenser.hasCoral()) {
        if (!hasHomedDispenser) {
          homeDispenser.schedule();
          hasHomedDispenser = true;
        }
      } else {
        hasHomedDispenser = false;
      }
    }

    // Tell elevator we are stowed
    elevator.setStowed(state == SuperstructureState.STOW);

    // Tell elevator if we have algae
    elevator.setHasAlgae(dispenser.hasAlgae() && !forceFastConstraints);

    // Tell dispenser if intaking
    dispenser.setIntaking(
        state == SuperstructureState.INTAKE || next == SuperstructureState.INTAKE);

    // E Stop Dispenser and Elevator if Necessary
    isEStopped =
        (isEStopped || elevator.isShouldEStop() || dispenser.isShouldEStop())
            && Constants.getMode() != Mode.SIM;
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

    // Update visualizer
    measuredVisualizer.update(
        elevator.getPositionMeters(),
        dispenser.getPivotAngle(),
        0.0,
        dispenser.hasAlgae(),
        dispenser.hasCoral());
    setpointVisualizer.update(
        elevator.getSetpoint().position,
        Rotation2d.fromRadians(dispenser.getSetpoint().position),
        0.0,
        dispenser.hasAlgae(),
        dispenser.hasCoral());
    goalVisualizer.update(
        elevator.getGoalMeters(),
        Rotation2d.fromRadians(dispenser.getGoal()),
        0.0,
        dispenser.hasAlgae(),
        dispenser.hasCoral());

    // Record cycle time
    LoggedTracer.record("Superstructure");
  }

  public void setOverrides(
      BooleanSupplier disableOverride, BooleanSupplier autoCoralStationIntakeOverride) {
    this.disableOverride = disableOverride;
    this.autoCoralStationIntakeOverride = autoCoralStationIntakeOverride;
  }

  public void adjustCoralThresholdOffset(double deltaDegrees) {
    dispenser.setCoralThresholdOffset(dispenser.getCoralThresholdOffset() + deltaDegrees);
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

  public void resetHasCoral() {
    dispenser.resetHasCoral();
  }

  public void resetHasAlgae() {
    dispenser.resetHasAlgae();
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

  public void setAutoStart() {
    state = SuperstructureState.AUTO_START;
    next = null;
    if (edgeCommand != null) {
      edgeCommand.getCommand().cancel();
    }
    dispenser.setHasCoral(true);
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

  /**
   * Run superstructure to {@link SuperstructureState} to while avoiding intake. Ends when all
   * subsystems are complete with profiles.
   */
  private EdgeCommand getEdgeCommand(SuperstructureState from, SuperstructureState to) {
    if (from == SuperstructureState.PRE_THROWN && to == SuperstructureState.THROWN) {
      Timer algaeEjectTimer = new Timer();
      return EdgeCommand.builder()
          .command(
              Commands.runOnce(
                      () -> {
                        algaeEjectTimer.restart();
                        forceFastConstraints = true;
                        elevator.setGoal(to.getValue().getPose().elevatorHeight());
                        dispenser.setGoal(to.getValue().getPose().pivotAngle());
                      })
                  .andThen(
                      Commands.waitUntil(
                          () ->
                              algaeEjectTimer.hasElapsed(
                                  AlgaeScoreCommands.throwGripperEjectTime.get())),
                      runSuperstructureExtras(SuperstructureState.THROWN),
                      Commands.runOnce(() -> forceFastConstraints = false)))
          .build();
    }

    BooleanSupplier fromIsLower =
        () ->
            from.getValue().getPose().pivotAngle().get().getDegrees()
                <= pivotSafeAngle.getDegrees();
    BooleanSupplier toIsLower =
        () ->
            to.getValue().getPose().pivotAngle().get().getDegrees() <= pivotSafeAngle.getDegrees();
    boolean passesThroughCrossMember = from.getValue().getHeight() != to.getValue().getHeight();
    if (passesThroughCrossMember) {
      SuperstructurePose safeSuperstructureSetpoint =
          new SuperstructurePose(
              () -> {
                double positionMeters = elevator.getPositionMeters();
                if (positionMeters >= stage2ToStage3Height) {
                  return stage2ToStage3Height + 0.1;
                } else if (positionMeters >= stage1ToStage2Height) {
                  return stage1ToStage2Height + 0.1;
                } else {
                  return MathUtil.clamp(positionMeters, 0.3, 0.5);
                }
              },
              () -> pivotSafeAngle);
      return EdgeCommand.builder()
          .command(
              runSuperstructurePose(safeSuperstructureSetpoint)
                  .andThen(Commands.waitUntil(dispenser::isAtGoal))
                  .onlyIf(fromIsLower)
                  .andThen(
                      Commands.either(
                          runElevator(to.getValue().getPose().elevatorHeight())
                              .andThen(
                                  Commands.waitUntil(elevator::isAtGoal),
                                  runSuperstructurePose(to.getValue().getPose()),
                                  Commands.waitUntil(this::mechanismsAtGoal)),
                          runSuperstructurePose(to.getValue().getPose())
                              .andThen(Commands.waitUntil(this::mechanismsAtGoal)),
                          toIsLower))
                  .deadlineFor(runSuperstructureExtras(to)))
          .build();
    } else {
      return EdgeCommand.builder()
          .command(
              runSuperstructurePose(to.getValue().getPose())
                  .andThen(Commands.waitUntil(this::mechanismsAtGoal))
                  .deadlineFor(runSuperstructureExtras(to)))
          .build();
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

  private Command getSlamCommand(Goal goal) {
    return Commands.runOnce(() -> {}); // Chariot does not exist right now :(
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
      FieldConstants.ReefLevel height, boolean eject) {
    return switch (height) {
      case L1 -> eject ? SuperstructureState.L1_CORAL_EJECT : SuperstructureState.L1_CORAL;
      case L2 -> eject ? SuperstructureState.L2_CORAL_EJECT : SuperstructureState.L2_CORAL;
      case L3 -> eject ? SuperstructureState.L3_CORAL_EJECT : SuperstructureState.L3_CORAL;
      case L4 -> eject ? SuperstructureState.L4_CORAL_EJECT : SuperstructureState.L4_CORAL;
    };
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

  @FunctionalInterface
  private interface QuintConsumer<A, B, C, D, E> {
    void accept(A a, B b, C c, D d, E e);
  }
}
