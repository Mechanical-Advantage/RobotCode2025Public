// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/Alert.h>
#include <frc2/command/Command.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>

#include <units/time.h>

#include <jgrapht/graph/DefaultDirectedGraph.hpp>
#include <jgrapht/graph/DefaultEdge.hpp>

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
#include "util/QuintConsumer.h"
#include "util/SuperstructureVisualizer.h"

#include "networktables/BooleanPublisher.h"
#include "networktables/DoublePublisher.h"
#include "networktables/IntegerPublisher.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StringPublisher.h"

class Superstructure
    : public frc2::CommandHelper<frc2::SubsystemBase, Superstructure> {
public:
  Superstructure(Elevator &elevator, Dispenser &dispenser);

  void Periodic() override;

  void SetOverrides(std::function<bool()> disableOverride,
                    std::function<bool()> autoCoralStationIntakeOverride);

  void AdjustCoralThresholdOffset(double deltaDegrees);

  bool AtGoal();

  bool HasAlgae();

  bool HasCoral();

  void ResetHasCoral();

  void ResetHasAlgae();

  frc2::CommandPtr RunGoal(SuperstructureState goal);

  frc2::CommandPtr RunGoal(std::function<SuperstructureState()> goal);

  void SetAutoStart();

  frc2::CommandPtr RunHomingSequence();

  frc2::CommandPtr SetCharacterizationMode();

  static SuperstructureState GetScoringState(FieldConstants::ReefLevel height,
                                             bool eject);

  bool IsEStopped() const { return isEStopped; }
  SuperstructureState GetState() const { return state; }
  SuperstructureState GetGoal() const { return goal; }
  bool GetRequestFunnelIntake() const { return requestFunnelIntake; }
  bool GetRequestFunnelOuttake() const { return requestFunnelOuttake; }

  struct EdgeCommand : public jgrapht::graph::DefaultEdge {
    frc2::CommandPtr command;
    bool restricted = false;
    AlgaeEdge algaeEdgeType = AlgaeEdge::NONE;

    EdgeCommand(frc2::CommandPtr command, bool restricted = false,
                AlgaeEdge algaeEdgeType = AlgaeEdge::NONE)
        : command(command), restricted(restricted),
          algaeEdgeType(algaeEdgeType) {}
  };

  enum class AlgaeEdge { NONE, NO_ALGAE, ALGAE };

private:
  Elevator &elevator;
  Dispenser &dispenser;

  jgrapht::graph::DefaultDirectedGraph<SuperstructureState, EdgeCommand> graph;

  EdgeCommand *edgeCommand = nullptr;

  SuperstructureState state = SuperstructureState::START;
  SuperstructureState next = SuperstructureState::START;
  SuperstructureState lastState = SuperstructureState::START;
  SuperstructureState sourceState = SuperstructureState::START;
  SuperstructureState goal = SuperstructureState::START;
  bool hasHomedDispenser = false;
  frc2::CommandPtr homeDispenser;

  bool isEStopped = false;

  networktables::BooleanPublisher characterizationModeOn =
      networktables::NetworkTableInstance::GetDefault()
          .GetBooleanTopic("/SmartDashboard/Characterization Mode On")
          .Publish();

  std::function<bool()> disableOverride = []() { return false; };
  std::function<bool()> autoCoralStationIntakeOverride = []() { return false; };
  frc::Alert driverDisableAlert =
      frc::Alert("Superstructure disabled due to driver override.",
                 frc::Alert::AlertType::kWarning);
  frc::Alert emergencyDisableAlert = frc::Alert(
      "Superstructure emergency disabled due to high position error. Disable "
      "the superstructure manually and reenable to reset.",
      frc::Alert::AlertType::kError);

  SuperstructureVisualizer measuredVisualizer =
      SuperstructureVisualizer("Measured");
  SuperstructureVisualizer setpointVisualizer =
      SuperstructureVisualizer("Setpoint");
  SuperstructureVisualizer goalVisualizer = SuperstructureVisualizer("Goal");

  bool requestFunnelIntake = false;
  bool requestFunnelOuttake = false;
  bool forceFastConstraints = false;

  void SetGoal(SuperstructureState goal);

  std::optional<SuperstructureState> Bfs(SuperstructureState start,
                                         SuperstructureState goal);

  EdgeCommand GetEdgeCommand(SuperstructureState from, SuperstructureState to);

  frc2::CommandPtr RunElevator(std::function<double()> elevatorHeight);

  frc2::CommandPtr
  RunDispenserPivot(std::function<frc::Rotation2d()> pivotAngle);

  frc2::CommandPtr RunSuperstructurePose(SuperstructurePose pose);

  frc2::CommandPtr RunSuperstructureExtras(SuperstructureState state);

  frc2::CommandPtr GetSlamCommand(Chariot::Goal goal);

  bool IsEdgeAllowed(EdgeCommand *edge, SuperstructureState goal);

  bool MechanismsAtGoal();
};