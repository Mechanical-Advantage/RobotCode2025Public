// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <map>
#include <string>

#include <frc/Alert.h>
#include <frc/DriverStation.h>
#include <frc/IterativeRobotBase.h>
#include <frc/PowerDistribution.h>
#include <frc/RobotController.h>
#include <frc/Timer.h>
#include <frc/Watchdog.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>

#include "BuildConstants.h"
#include "Constants.h"
#include "FieldConstants.h"
#include "RobotContainer.h"
#include "junction/LoggedRobot.h"
#include "junction/inputs/LoggedPowerDistribution.h"
#include "junction/wpilog/WPILOGReader.h"
#include "junction/wpilog/WPILOGWriter.h"
#include "subsystems/leds/Leds.h"
#include "util/CanivoreReader.h"
#include "util/LoggedTracer.h"
#include "util/NTClientLogger.h"
#include "util/PhoenixUtil.h"
#include "util/SystemTimeValidReader.h"
#include "util/VirtualSubsystem.h"
#include "util/rlog/RLOGServer.h"

class Robot : public JUnction::LoggedRobot {
public:
  Robot();

  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  static bool IsJITing();

private:
  static constexpr double loopOverrunWarningTimeout = 0.2;
  static constexpr double canErrorTimeThreshold = 0.5;
  static constexpr double canivoreErrorTimeThreshold = 0.5;
  static constexpr double lowBatteryVoltage = 11.8;
  static constexpr double lowBatteryDisabledTime = 1.5;
  static constexpr double lowBatteryMinCycleCount = 10;

  int lowBatteryCycleCount = 0;

  frc2::command::Command *autonomousCommand = nullptr;
  RobotContainer *robotContainer = nullptr;
  double autoStart = 0.0;
  bool autoMessagePrinted = false;
  frc::Timer canInitialErrorTimer;
  frc::Timer canErrorTimer;
  frc::Timer canivoreErrorTimer;
  frc::Timer disabledTimer;
  CanivoreReader canivoreReader{"*"};

  frc::Alert canErrorAlert{
      "CAN errors detected, robot may not be controllable.",
      frc::Alert::AlertType::kError};
  frc::Alert canivoreErrorAlert{
      "CANivore errors detected, robot may not be controllable.",
      frc::Alert::AlertType::kError};
  frc::Alert lowBatteryAlert{"Battery voltage is very low, consider turning "
                             "off the robot or replacing the battery.",
                             frc::Alert::AlertType::kWarning};
  frc::Alert jitAlert{"Please wait to enable, JITing in progress.",
                      frc::Alert::AlertType::kWarning};

  std::map<std::string, int> commandCounts;

  void LogCommand(frc2::command::Command *command, bool active);
};