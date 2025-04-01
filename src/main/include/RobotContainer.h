// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <optional>
#include <vector>

#include <frc/Alert.h>
#include <frc/DriverStation.h>
#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc2/command/button/Trigger.h>

#include "Constants.h"
#include "FieldConstants.h"
#include "commands/AlgaeScoreCommands.h"
#include "commands/AutoScoreCommands.h"
#include "commands/DriveCommands.h"
#include "commands/DriveTrajectory.h"
#include "commands/IntakeCommands.h"
#include "junction/networktables/LoggedDashboardChooser.h"
#include "junction/networktables/LoggedNetworkNumber.h"
#include "subsystems/climber/Climber.h"
#include "subsystems/climber/ClimberIO.h"
#include "subsystems/climber/ClimberIOSim.h"
#include "subsystems/climber/ClimberIOTalonFX.h"
#include "subsystems/drive/Drive.h"
#include "subsystems/drive/GyroIO.h"
#include "subsystems/drive/GyroIOPigeon2.h"
#include "subsystems/drive/GyroIORedux.h"
#include "subsystems/drive/ModuleIO.h"
#include "subsystems/drive/ModuleIOComp.h"
#include "subsystems/drive/ModuleIODev.h"
#include "subsystems/drive/ModuleIOSim.h"
#include "subsystems/drive/trajectory/HolonomicTrajectory.h"
#include "subsystems/leds/Leds.h"
#include "subsystems/objectivetracker/ObjectiveTracker.h"
#include "subsystems/objectivetracker/ReefControlsIO.h"
#include "subsystems/objectivetracker/ReefControlsIOServer.h"
#include "subsystems/rollers/RollerSystem.h"
#include "subsystems/rollers/RollerSystemIO.h"
#include "subsystems/rollers/RollerSystemIOSim.h"
#include "subsystems/rollers/RollerSystemIOSpark.h"
#include "subsystems/rollers/RollerSystemIOTalonFX.h"
#include "subsystems/superstructure/Superstructure.h"
#include "subsystems/superstructure/dispenser/Dispenser.h"
#include "subsystems/superstructure/dispenser/PivotIO.h"
#include "subsystems/superstructure/dispenser/PivotIOSim.h"
#include "subsystems/superstructure/dispenser/PivotIOTalonFX.h"
#include "subsystems/superstructure/elevator/Elevator.h"
#include "subsystems/superstructure/elevator/ElevatorIO.h"
#include "subsystems/superstructure/elevator/ElevatorIOSim.h"
#include "subsystems/superstructure/elevator/ElevatorIOTalonFX.h"
#include "subsystems/superstructure/sensors/CoralSensorIO.h"
#include "subsystems/superstructure/sensors/CoralSensorIOLaserCan.h"
#include "subsystems/vision/Vision.h"
#include "subsystems/vision/VisionIO.h"
#include "subsystems/vision/VisionIONorthstar.h"
#include "util/Container.h"
#include "util/DoublePressTracker.h"
#include "util/MirrorUtil.h"
#include "util/OverrideSwitches.h"
#include "util/RobotState.h"
#include "util/TriggerUtil.h"

class RobotContainer {
public:
  RobotContainer();

  frc2::command::Command *getAutonomousCommand();
  void updateDashboardOutputs();
  void updateAlerts();
  FieldConstants::AprilTagLayoutType getSelectedAprilTagLayout();

private:
  void configureButtonBindings();
  frc2::command::Command *controllerRumbleCommand();

  Drive *drive = nullptr;
  Vision *vision = nullptr;
  Superstructure *superstructure = nullptr;
  RollerSystem *funnel = nullptr;
  Climber *climber = nullptr;
  ObjectiveTracker *objectiveTracker = nullptr;
  Leds &leds = Leds::GetInstance();

  frc2::command::button::CommandXboxController driver{0};
  frc2::command::button::CommandXboxController operatorController{1};
  OverrideSwitches overrides{5};
  frc2::command::button::Trigger robotRelative = overrides.driverSwitch(0);
  frc2::command::button::Trigger superstructureDisable =
      overrides.driverSwitch(1);
  frc2::command::button::Trigger superstructureCoast =
      overrides.driverSwitch(2);
  frc2::command::button::Trigger disableAutoCoralStationIntake =
      overrides.operatorSwitch(0).Negate();
  frc2::command::button::Trigger disableReefAutoAlign =
      overrides.operatorSwitch(1);
  frc2::command::button::Trigger disableCoralStationAutoAlign =
      overrides.operatorSwitch(2);
  frc2::command::button::Trigger disableAlgaeScoreAutoAlign =
      overrides.operatorSwitch(3);
  frc2::command::button::Trigger disableDispenserGamePieceDetection =
      overrides.operatorSwitch(4);

  frc2::command::button::Trigger aprilTagsReef =
      overrides.multiDirectionSwitchLeft();
  frc2::command::button::Trigger aprilTagFieldBorder =
      overrides.multiDirectionSwitchRight();
  frc::Alert aprilTagLayoutAlert{"", frc::Alert::AlertType::kInfo};
  frc::Alert driverDisconnected{"Driver controller disconnected (port 0).",
                                frc::Alert::AlertType::kWarning};
  frc::Alert operatorDisconnected{"Operator controller disconnected (port 1).",
                                  frc::Alert::AlertType::kWarning};
  frc::Alert overrideDisconnected{"Override controller disconnected (port 5).",
                                  frc::Alert::AlertType::kInfo};
  LoggedNetworkNumber endgameAlert1{"/SmartDashboard/Endgame Alert #1", 30.0};
  LoggedNetworkNumber endgameAlert2{"/SmartDashboard/Endgame Alert #2", 15.0};

  bool superstructureCoastOverride = false;

  LoggedDashboardChooser<frc2::command::Command *> autoChooser{"Auto Choices"};
};