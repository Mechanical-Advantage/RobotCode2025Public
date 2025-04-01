// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/superstructure/Superstructure.h"

#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/Alert.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>

#include <units/time.h>

#include "Constants.h"
#include "FieldConstants.h"
#include "RobotState.h"
#include "commands/AlgaeScoreCommands.h"
#include "subsystems/leds/Leds.h"
#include "subsystems/superstructure/SuperstructureConstants.h"
#include "subsystems/superstructure/chariot/Chariot.h"
#include "subsystems/superstructure/dispenser/Dispenser.h"
#include "subsystems/superstructure/elevator/Elevator.h"
#include "util/AllianceFlipUtil.h"
#include "util/LoggedTracer.h"
#include "util/SuperstructureVisualizer.h"

#include "networktables/NetworkTableInstance.h"

Superstructure::Superstructure(Elevator &elevator, Dispenser &dispenser)
    : elevator(elevator), dispenser(dispenser),
      graph(jgrapht::graph::DefaultDirectedGraph<SuperstructureState,
                                                 EdgeCommand>()),
      homeDispenser(dispenser.HomingSequence()) {
  frc2::Trigger([this]() {
    return disableOverride();
  }).OnFalse(frc2::Commands::RunOnce([this]() {
               isEStopped = false;
             }).IgnoringDisable(true));

  for (auto state : {SuperstructureState::START,
                     SuperstructureState::STOW,
                     SuperstructureState::INTAKE,
                     SuperstructureState::GOODBYE_CORAL,
                     SuperstructureState::GOODBYE_CORAL_EJECT,
                     SuperstructureState::L1_CORAL,
                     SuperstructureState::L1_CORAL_EJECT,
                     SuperstructureState::L2_CORAL,
                     SuperstructureState::L2_CORAL_EJECT,
                     SuperstructureState::L3_CORAL,
                     SuperstructureState::L3_CORAL_EJECT,
                     SuperstructureState::L4_CORAL,
                     SuperstructureState::L4_CORAL_EJECT,
                     SuperstructureState::ALGAE_STOW,
                     SuperstructureState::ALGAE_STOW_INTAKE,
                     SuperstructureState::ALGAE_L2_INTAKE,
                     SuperstructureState::ALGAE_L3_INTAKE,
                     SuperstructureState::PROCESSED,
                     SuperstructureState::PRE_TOSS,
                     SuperstructureState::TOSS,
                     SuperstructureState::PRE_THROWN,
                     SuperstructureState::THROWN,
                     SuperstructureState::AUTO_START,
                     SuperstructureState::CHARACTERIZATION}) {
    graph.addVertex(state);
  }

  graph.addEdge(
      SuperstructureState::START, SuperstructureState::STOW,
      EdgeCommand(
          RunSuperstructureExtras(SuperstructureState::STOW)
              .AndThen(
                  frc2::Commands::Parallel(dispenser.HomingSequence(),
                                           frc2::Commands::Wait(0.2_s).AndThen(
                                               elevator.HomingSequence()))
                      .DeadlineWith(frc2::Commands::StartEnd(
                          [this]() { requestFunnelOuttake = true; },
                          [this]() { requestFunnelOuttake = false; })),
                  RunSuperstructurePose(
                      SuperstructureState::STOW.GetValue().GetPose()),
                  frc2::Commands::WaitUntil(
                      [this]() { return MechanismsAtGoal(); }))));

  graph.addEdge(
      SuperstructureState::AUTO_START, SuperstructureState::STOW,
      EdgeCommand(
          frc2::Commands::RunOnce([this]() { elevator.SetHome(); })
              .AndThen(RunSuperstructurePose(
                           SuperstructurePose::Preset::STOW.GetPose()),
                       frc2::Commands::WaitUntil(
                           [this]() { return MechanismsAtGoal(); }),
                       RunSuperstructureExtras(SuperstructureState::STOW))));
  graph.addEdge(SuperstructureState::CHARACTERIZATION,
                SuperstructureState::STOW,
                EdgeCommand(frc2::Commands::Idle(*this).Until(
                    [this]() { return !characterizationModeOn.Get(); })));

  const std::set<SuperstructureState> freeNoAlgaeStates = {
      SuperstructureState::STOW,
      SuperstructureState::GOODBYE_CORAL,
      SuperstructureState::L1_CORAL,
      SuperstructureState::L2_CORAL,
      SuperstructureState::L3_CORAL,
      SuperstructureState::L4_CORAL,
      SuperstructureState::ALGAE_STOW_INTAKE,
      SuperstructureState::ALGAE_L2_INTAKE,
      SuperstructureState::ALGAE_L3_INTAKE};

  const std::set<SuperstructureState> freeAlgaeStates = {
      SuperstructureState::ALGAE_STOW,
      SuperstructureState::ALGAE_STOW_INTAKE,
      SuperstructureState::ALGAE_L2_INTAKE,
      SuperstructureState::ALGAE_L3_INTAKE,
      SuperstructureState::PRE_TOSS,
      SuperstructureState::PRE_THROWN};

  const std::set<SuperstructureState> algaeIntakeStates = {
      SuperstructureState::ALGAE_STOW_INTAKE,
      SuperstructureState::ALGAE_L2_INTAKE,
      SuperstructureState::ALGAE_L3_INTAKE};

  for (auto from : freeNoAlgaeStates) {
    for (auto to : freeNoAlgaeStates) {
      if (from == to)
        continue;
      if (algaeIntakeStates.count(from) && algaeIntakeStates.count(to))
        continue;
      if (algaeIntakeStates.count(from)) {
        graph.addEdge(
            from, to,
            GetEdgeCommand(from, to).algaeEdgeType(AlgaeEdge::NO_ALGAE));
      } else {
        graph.addEdge(from, to, GetEdgeCommand(from, to));
      }
    }
  }

  for (auto from : freeAlgaeStates) {
    for (auto to : freeAlgaeStates) {
      if (from == to)
        continue;
      if (algaeIntakeStates.count(from) && algaeIntakeStates.count(to))
        continue;
      if (algaeIntakeStates.count(from)) {
        graph.addEdge(from, to,
                      GetEdgeCommand(from, to).algaeEdgeType(AlgaeEdge::ALGAE));
      } else {
        graph.addEdge(from, to, GetEdgeCommand(from, to));
      }
    }
  }

  for (auto from : algaeIntakeStates) {
    for (auto to : algaeIntakeStates) {
      if (from == to)
        continue;
      graph.addEdge(from, to, GetEdgeCommand(from, to));
    }
  }

  auto addEdge = [this](SuperstructureState from, SuperstructureState to,
                        bool restricted, AlgaeEdge algaeEdgeType,
                        bool reverse) {
    graph.addEdge(from, to,
                  GetEdgeCommand(from, to)
                      .restricted(restricted)
                      .algaeEdgeType(algaeEdgeType));
    if (reverse) {
      graph.addEdge(to, from,
                    GetEdgeCommand(to, from)
                        .restricted(restricted)
                        .algaeEdgeType(algaeEdgeType));
    }
  };

  const std::set<std::pair<SuperstructureState, SuperstructureState>>
      pairedStates = {
          {SuperstructureState::STOW, SuperstructureState::INTAKE},
          {SuperstructureState::GOODBYE_CORAL,
           SuperstructureState::GOODBYE_CORAL_EJECT},
          {SuperstructureState::L1_CORAL, SuperstructureState::L1_CORAL_EJECT},
          {SuperstructureState::L2_CORAL, SuperstructureState::L2_CORAL_EJECT},
          {SuperstructureState::L3_CORAL, SuperstructureState::L3_CORAL_EJECT},
          {SuperstructureState::L4_CORAL, SuperstructureState::L4_CORAL_EJECT},
          {SuperstructureState::PRE_TOSS, SuperstructureState::TOSS}};
  for (const auto &pair : pairedStates) {
    addEdge(pair.first, pair.second, false, AlgaeEdge::NONE, true);
  }

  const std::set<SuperstructureState> recoverableAlgaeStates = {
      SuperstructureState::ALGAE_STOW, SuperstructureState::PROCESSED,
      SuperstructureState::PRE_THROWN, SuperstructureState::PRE_TOSS,
      SuperstructureState::TOSS,       SuperstructureState::THROWN};
  for (auto from : recoverableAlgaeStates) {
    for (auto to : freeNoAlgaeStates) {
      graph.addEdge(
          from, to,
          GetEdgeCommand(from, to).algaeEdgeType(AlgaeEdge::NO_ALGAE));
    }
  }

  addEdge(SuperstructureState::ALGAE_STOW, SuperstructureState::PROCESSED, true,
          AlgaeEdge::NONE, false);
  addEdge(SuperstructureState::PROCESSED, SuperstructureState::ALGAE_STOW,
          false, AlgaeEdge::ALGAE, false);
  addEdge(SuperstructureState::THROWN, SuperstructureState::ALGAE_STOW, false,
          AlgaeEdge::ALGAE, false);
  addEdge(SuperstructureState::PRE_TOSS, SuperstructureState::ALGAE_STOW, false,
          AlgaeEdge::ALGAE, false);
  addEdge(SuperstructureState::STOW, SuperstructureState::ALGAE_STOW, false,
          AlgaeEdge::ALGAE, false);
  addEdge(SuperstructureState::ALGAE_STOW, SuperstructureState::STOW, false,
          AlgaeEdge::NO_ALGAE, false);
  addEdge(SuperstructureState::PRE_THROWN, SuperstructureState::THROWN, true,
          AlgaeEdge::NONE, false);
  addEdge(SuperstructureState::THROWN, SuperstructureState::PRE_THROWN, false,
          AlgaeEdge::ALGAE, false);

  SetDefaultCommand(
      RunGoal([this]() {
        frc::Pose2d robot = AllianceFlipUtil::Apply(
            RobotState::GetInstance().GetEstimatedPose());
        if (!dispenser.HasCoral() && !dispenser.HasAlgae() &&
            robot.X() < FieldConstants::fieldLength / 5.0 &&
            (robot.Y() < FieldConstants::fieldWidth / 5.0 ||
             robot.Y() > FieldConstants::fieldWidth * 4.0 / 5.0) &&
            !autoCoralStationIntakeOverride()) {
          if (state == SuperstructureState::INTAKE) {
            requestFunnelIntake = true;
          }
          return SuperstructureState::INTAKE;
        }
        requestFunnelIntake = false;
        return dispenser.HasAlgae() ? SuperstructureState::ALGAE_STOW
                                    : SuperstructureState::STOW;
      }).FinallyDo([this]() { requestFunnelIntake = false; }));
}

void Superstructure::Periodic() {
  elevator.Periodic();
  dispenser.Periodic();

  if (characterizationModeOn.Get()) {
    state = SuperstructureState::CHARACTERIZATION;
    next = SuperstructureState::START;
    Leds::GetInstance().characterizationMode = true;
  } else {
    Leds::GetInstance().characterizationMode = false;
  }

  if (frc::DriverStation::IsDisabled()) {
    next = SuperstructureState::START;
  } else if (edgeCommand == nullptr || !edgeCommand->command->IsScheduled()) {
    if (next != SuperstructureState::START) {
      state = next;
      next = SuperstructureState::START;
    }

    if (state != goal) {
      Bfs(state, goal).if_exists([this](SuperstructureState nextState) {
        next = nextState;
        edgeCommand = &graph.getEdge(state, next);
        edgeCommand->command->Schedule();
      });
    }

    if (state != lastState) {
      sourceState = lastState;
      lastState = state;
    }

    if (state == SuperstructureState::STOW &&
        goal == SuperstructureState::STOW &&
        sourceState != SuperstructureState::INTAKE && !dispenser.HasCoral()) {
      if (!hasHomedDispenser) {
        homeDispenser->Schedule();
        hasHomedDispenser = true;
      }
    } else {
      hasHomedDispenser = false;
    }
  }

  elevator.SetStowed(state == SuperstructureState::STOW);
  elevator.SetHasAlgae(dispenser.HasAlgae() && !forceFastConstraints);
  dispenser.SetIntaking(state == SuperstructureState::INTAKE ||
                        next == SuperstructureState::INTAKE);

  isEStopped =
      (isEStopped || elevator.IsShouldEStop() || dispenser.IsShouldEStop()) &&
      Constants::GetMode() != Mode::SIM;
  elevator.SetEStopped(isEStopped);
  dispenser.SetEStopped(isEStopped);

  driverDisableAlert.Set(disableOverride());
  emergencyDisableAlert.Set(isEStopped);
  Leds::GetInstance().superstructureEstopped = isEStopped;

  Logger::RecordOutput("Superstructure/State", state);
  Logger::RecordOutput("Superstructure/Next", next);
  Logger::RecordOutput("Superstructure/Goal", goal);
  if (edgeCommand != nullptr) {
    Logger::RecordOutput(
        "Superstructure/EdgeCommand",
        static_cast<int>(graph.getEdgeSource(*edgeCommand)) + " --> " +
            static_cast<int>(graph.getEdgeTarget(*edgeCommand)));
  } else {
    Logger::RecordOutput("Superstructure/EdgeCommand", "");
  }

  measuredVisualizer.Update(elevator.GetPositionMeters(),
                            dispenser.GetPivotAngle(), 0.0,
                            dispenser.HasAlgae());
  setpointVisualizer.Update(
      elevator.GetSetpoint().position,
      frc::Rotation2d::Radians(dispenser.GetSetpoint().position), 0.0,
      dispenser.HasAlgae());
  goalVisualizer.Update(elevator.GetGoalMeters(),
                        frc::Rotation2d::Radians(dispenser.GetGoal()), 0.0,
                        dispenser.HasAlgae());

  LoggedTracer::Record("Superstructure");
}

void Superstructure::SetOverrides(
    std::function<bool()> disableOverride,
    std::function<bool()> autoCoralStationIntakeOverride) {
  this->disableOverride = disableOverride;
  this->autoCoralStationIntakeOverride = autoCoralStationIntakeOverride;
}

void Superstructure::AdjustCoralThresholdOffset(double deltaDegrees) {
  dispenser.SetCoralThresholdOffset(dispenser.GetCoralThresholdOffset() +
                                    deltaDegrees);
}

bool Superstructure::AtGoal() { return state == goal; }

bool Superstructure::HasAlgae() { return dispenser.HasAlgae(); }

bool Superstructure::HasCoral() { return dispenser.HasCoral(); }

void Superstructure::ResetHasCoral() { dispenser.ResetHasCoral(); }

void Superstructure::ResetHasAlgae() { dispenser.ResetHasAlgae(); }

void Superstructure::SetGoal(SuperstructureState goal) {
  if (this->goal == goal)
    return;
  this->goal = goal;

  if (next == SuperstructureState::START)
    return;

  EdgeCommand *edgeToCurrentState = &graph.getEdge(next, state);
  if (edgeCommand->command->IsScheduled() && edgeToCurrentState != nullptr &&
      IsEdgeAllowed(edgeToCurrentState, goal)) {
    Bfs(state, goal).if_exists([this](SuperstructureState newNext) {
      if (newNext == next) {
        return;
      }

      if (newNext != state && graph.getEdge(next, newNext).command != nullptr) {
        edgeCommand->command->Cancel();
        edgeCommand = &graph.getEdge(state, newNext);
        edgeCommand->command->Schedule();
        next = newNext;
      } else {
        edgeCommand->command->Cancel();
        edgeCommand = &graph.getEdge(next, state);
        edgeCommand->command->Schedule();
        auto temp = state;
        state = next;
        next = temp;
      }
    });
  }
}

frc2::CommandPtr Superstructure::RunGoal(SuperstructureState goal) {
  return frc2::Commands::RunOnce([this, goal]() { SetGoal(goal); })
      .AndThen(frc2::Commands::Idle(*this));
}

frc2::CommandPtr
Superstructure::RunGoal(std::function<SuperstructureState()> goal) {
  return frc2::Commands::Run([this, goal]() { SetGoal(goal()); });
}

void Superstructure::SetAutoStart() {
  state = SuperstructureState::AUTO_START;
  next = SuperstructureState::START;
  if (edgeCommand != nullptr) {
    edgeCommand->command->Cancel();
  }
  dispenser.SetHasCoral(true);
}

std::optional<SuperstructureState>
Superstructure::Bfs(SuperstructureState start, SuperstructureState goal) {
  std::map<SuperstructureState, SuperstructureState> parents;
  std::queue<SuperstructureState> queue;
  queue.push(start);
  parents[start] = SuperstructureState::START;

  while (!queue.empty()) {
    SuperstructureState current = queue.front();
    queue.pop();

    if (current == goal) {
      break;
    }

    for (auto edge : graph.outgoingEdgesOf(current)) {
      if (IsEdgeAllowed(&edge, goal)) {
        SuperstructureState neighbor = graph.getEdgeTarget(edge);
        if (parents.find(neighbor) == parents.end()) {
          parents[neighbor] = current;
          queue.push(neighbor);
        }
      }
    }
  }

  if (parents.find(goal) == parents.end()) {
    return std::nullopt;
  }

  SuperstructureState nextState = goal;
  while (nextState != start) {
    SuperstructureState parent = parents[nextState];
    if (parent == SuperstructureState::START) {
      return nextState;
    } else if (parent == start) {
      return nextState;
    }
    nextState = parent;
  }
  return nextState;
}

Superstructure::EdgeCommand
Superstructure::GetEdgeCommand(SuperstructureState from,
                               SuperstructureState to) {
  if (from == SuperstructureState::PRE_THROWN &&
      to == SuperstructureState::THROWN) {
    frc::Timer algaeEjectTimer;
    return EdgeCommand(
        frc2::Commands::RunOnce([this, to, &algaeEjectTimer]() {
          algaeEjectTimer.Restart();
          forceFastConstraints = true;
          elevator.SetGoal(to.GetValue().GetPose().elevatorHeight());
          dispenser.SetGoal(to.GetValue().GetPose().pivotAngle());
        })
            .AndThen(frc2::Commands::WaitUntil([this, &algaeEjectTimer]() {
                       return algaeEjectTimer.HasElapsed(
                           AlgaeScoreCommands::throwGripperEjectTime);
                     }),
                     RunSuperstructureExtras(SuperstructureState::THROWN),
                     frc2::Commands::RunOnce(
                         [this]() { forceFastConstraints = false; })));
  }

  auto fromIsLower = [from]() {
    return from.GetValue().GetPose().pivotAngle().Degrees() <=
           pivotSafeAngle.Degrees();
  };
  auto toIsLower = [to]() {
    return to.GetValue().GetPose().pivotAngle().Degrees() <=
           pivotSafeAngle.Degrees();
  };
  bool passesThroughCrossMember =
      from.GetValue().GetHeight() != to.GetValue().GetHeight();
  if (passesThroughCrossMember) {
    SuperstructurePose safeSuperstructureSetpoint = SuperstructurePose(
        [this]() {
          double positionMeters = elevator.GetPositionMeters();
          if (positionMeters >= stage2ToStage3Height) {
            return stage2ToStage3Height + 0.1;
          } else if (positionMeters >= stage1ToStage2Height) {
            return stage1ToStage2Height + 0.1;
          } else {
            return std::clamp(positionMeters, 0.3, 0.5);
          }
        },
        []() { return pivotSafeAngle; });
    return EdgeCommand(
        RunSuperstructurePose(safeSuperstructureSetpoint)
            .AndThen(frc2::Commands::WaitUntil(
                [this]() { return dispenser.IsAtGoal(); }))
            .OnlyIf(fromIsLower)
            .AndThen(frc2::Commands::Either(
                RunElevator(
                    [to]() { return to.GetValue().GetPose().elevatorHeight(); })
                    .AndThen(frc2::Commands::WaitUntil(
                                 [this]() { return elevator.IsAtGoal(); }),
                             RunSuperstructurePose(to.GetValue().GetPose()),
                             frc2::Commands::WaitUntil(
                                 [this]() { return MechanismsAtGoal(); })),
                RunSuperstructurePose(to.GetValue().GetPose())
                    .AndThen(frc2::Commands::WaitUntil(
                        [this]() { return MechanismsAtGoal(); })),
                toIsLower))
            .DeadlineWith(RunSuperstructureExtras(to)));
  } else {
    return EdgeCommand(RunSuperstructurePose(to.GetValue().GetPose())
                           .AndThen(frc2::Commands::WaitUntil(
                               [this]() { return MechanismsAtGoal(); }))
                           .DeadlineWith(RunSuperstructureExtras(to)));
  }
}

frc2::CommandPtr Superstructure::RunHomingSequence() {
  return frc2::Commands::RunOnce([this]() {
    state = SuperstructureState::START;
    next = SuperstructureState::START;
    if (edgeCommand != nullptr) {
      edgeCommand->command->Cancel();
    }
  });
}

frc2::CommandPtr Superstructure::SetCharacterizationMode() {
  return frc2::Commands::RunOnce([this]() {
    state = SuperstructureState::CHARACTERIZATION;
    characterizationModeOn.Set(true);
    next = SuperstructureState::START;
    if (edgeCommand != nullptr) {
      edgeCommand->command->Cancel();
    }
  });
}

frc2::CommandPtr
Superstructure::RunElevator(std::function<double()> elevatorHeight) {
  return frc2::Commands::RunOnce(
      [this, elevatorHeight]() { elevator.SetGoal(elevatorHeight()); });
}

frc2::CommandPtr
Superstructure::RunDispenserPivot(std::function<frc::Rotation2d()> pivotAngle) {
  return frc2::Commands::RunOnce(
      [this, pivotAngle]() { dispenser.SetGoal(pivotAngle()); });
}

frc2::CommandPtr
Superstructure::RunSuperstructurePose(SuperstructurePose pose) {
  return RunElevator(pose.elevatorHeight())
      .AlongWith(RunDispenserPivot(pose.pivotAngle()));
}

frc2::CommandPtr
Superstructure::RunSuperstructureExtras(SuperstructureState state) {
  return frc2::Commands::RunOnce([this, state]() {
    dispenser.SetTunnelVolts(state.GetValue().GetTunnelVolts());
    dispenser.SetGripperGoal(state.GetValue().GetGripperGoal());
  });
}

frc2::CommandPtr Superstructure::GetSlamCommand(Chariot::Goal goal) {
  return frc2::Commands::RunOnce([]() {});
}

bool Superstructure::IsEdgeAllowed(EdgeCommand *edge,
                                   SuperstructureState goal) {
  return (!edge->restricted || goal == graph.getEdgeTarget(*edge)) &&
         (edge->algaeEdgeType == AlgaeEdge::NONE ||
          dispenser.HasAlgae() == (edge->algaeEdgeType == AlgaeEdge::ALGAE));
}

bool Superstructure::MechanismsAtGoal() {
  return elevator.IsAtGoal() &&
         (dispenser.IsAtGoal() || Constants::GetRobot() == RobotType::DEVBOT);
}

SuperstructureState
Superstructure::GetScoringState(FieldConstants::ReefLevel height, bool eject) {
  switch (height) {
  case FieldConstants::ReefLevel::L1:
    return eject ? SuperstructureState::L1_CORAL_EJECT
                 : SuperstructureState::L1_CORAL;
  case FieldConstants::ReefLevel::L2:
    return eject ? SuperstructureState::L2_CORAL_EJECT
                 : SuperstructureState::L2_CORAL;
  case FieldConstants::ReefLevel::L3:
    return eject ? SuperstructureState::L3_CORAL_EJECT
                 : SuperstructureState::L3_CORAL;
  case FieldConstants::ReefLevel::L4:
    return eject ? SuperstructureState::L4_CORAL_EJECT
                 : SuperstructureState::L4_CORAL;
  }
  return SuperstructureState::STOW;
}
