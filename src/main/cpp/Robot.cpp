// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "Robot.h"

#include <chrono>
#include <map>
#include <string>

#include <ctre/phoenix6/hardware/TalonFX.h>
#include <frc/Alert.h>
#include <frc/DriverStation.h>
#include <frc/IterativeRobotBase.h>
#include <frc/PowerDistribution.h>
#include <frc/RobotController.h>
#include <frc/Threads.h>
#include <frc/Timer.h>
#include <frc/Watchdog.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>

#include "BuildConstants.h"
#include "Constants.h"
#include "FieldConstants.h"
#include "RobotContainer.h"
#include "junction/AutoLogOutputManager.h"
#include "junction/LogFileUtil.h"
#include "junction/Logger.h"
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

Robot::Robot() {
  // Reset encoder on pivot
  if (Constants::GetRobot() == Constants::Robot::kCompbot) {
    ctre::phoenix6::hardware::TalonFX pivotTalon{5};
    PhoenixUtil::TryUntilOk(15, [&]() { return pivotTalon.SetPosition(0.0); });
    pivotTalon.Close();
  }

  // Start loading animation
  Leds::GetInstance();

  // Record metadata
  JUnction::Logger::RecordMetadata("Robot", Constants::GetRobot().ToString());
  JUnction::Logger::RecordMetadata("TuningMode",
                                   std::to_string(Constants::tuningMode));
  JUnction::Logger::RecordMetadata("FieldType",
                                   FieldConstants::fieldType.ToString());
  JUnction::Logger::RecordMetadata("RuntimeType", GetRuntimeType().ToString());
  JUnction::Logger::RecordMetadata("ProjectName", BuildConstants::MAVEN_NAME);
  JUnction::Logger::RecordMetadata("BuildDate", BuildConstants::BUILD_DATE);
  JUnction::Logger::RecordMetadata("GitSHA", BuildConstants::GIT_SHA);
  JUnction::Logger::RecordMetadata("GitDate", BuildConstants::GIT_DATE);
  JUnction::Logger::RecordMetadata("GitBranch", BuildConstants::GIT_BRANCH);
  switch (BuildConstants::DIRTY) {
  case 0:
    JUnction::Logger::RecordMetadata("GitDirty", "All changes committed");
    break;
  case 1:
    JUnction::Logger::RecordMetadata("GitDirty", "Uncomitted changes");
    break;
  default:
    JUnction::Logger::RecordMetadata("GitDirty", "Unknown");
    break;
  }

  // Set up data receivers & replay source
  switch (Constants::GetMode()) {
  case Constants::Mode::kReal:
    // Running on a real robot, log to a USB stick ("/U/logs")
    JUnction::Logger::AddDataReceiver(new JUnction::WPILOGWriter());
    JUnction::Logger::AddDataReceiver(new rlog::RLOGServer());
    if (Constants::GetRobot() == Constants::Robot::kCompbot) {
      JUnction::LoggedPowerDistribution::GetInstance(
          50, frc::PowerDistribution::ModuleType::kRev);
    }
    break;

  case Constants::Mode::kSim:
    // Running a physics simulator, log to NT
    JUnction::Logger::AddDataReceiver(new rlog::RLOGServer());
    break;

  case Constants::Mode::kReplay:
    // Replaying a log, set up replay source
    SetUseTiming(false); // Run as fast as possible
    std::string logPath = JUnction::LogFileUtil::FindReplayLog();
    JUnction::Logger::SetReplaySource(new JUnction::WPILOGReader(logPath));
    JUnction::Logger::AddDataReceiver(new JUnction::WPILOGWriter(
        JUnction::LogFileUtil::AddPathSuffix(logPath, "_sim")));
    break;
  }

  // Set up auto logging for RobotState
  JUnction::AutoLogOutputManager::AddObject(RobotState::GetInstance());

  // Start AdvantageKit logger
  JUnction::Logger::Start();

  // Adjust loop overrun warning timeout
  try {
    std::unique_ptr<frc::Watchdog> watchdog;
    std::unique_ptr<std::remove_pointer_t<decltype(this)>> robotPtr(this);
    std::unique_ptr<frc::IterativeRobotBase> robotBasePtr(
        dynamic_cast<frc::IterativeRobotBase *>(robotPtr.release()));

    std::unique_ptr<std::remove_pointer_t<decltype(robotBasePtr.get())>>
        robotBase(robotBasePtr.release());

    std::unique_ptr<std::remove_pointer_t<decltype(robotBase.get())>> robot(
        robotBase.release());

    std::unique_ptr<std::remove_pointer_t<decltype(robot.get())>> iterRobot(
        robot.release());

    std::unique_ptr<std::remove_pointer_t<decltype(iterRobot.get())>>
        iterRobotBasePtr(iterRobot.release());

    std::unique_ptr<frc::IterativeRobotBase> iterRobotBase(
        iterRobotBasePtr.release());

    std::unique_ptr<std::remove_pointer_t<decltype(iterRobotBase.get())>>
        iterRobotBase2(iterRobotBase.release());

    watchdog.reset(iterRobotBase2->GetWatchdog());

    watchdog->SetTimeout(loopOverrunWarningTimeout);
  } catch (const std::exception &e) {
    frc::DriverStation::ReportWarning(
        "Failed to disable loop overrun warnings.", false);
  }

  // Start system time valid reader
  SystemTimeValidReader::Start();

  // Rely on our custom alerts for disconnected controllers
  frc::DriverStation::SilenceJoystickConnectionWarning(true);

  // Log active commands
  frc2::command::CommandScheduler::GetInstance().onCommandInitialize.connect(
      [this](frc2::command::Command *command) { LogCommand(command, true); });
  frc2::command::CommandScheduler::GetInstance().onCommandFinish.connect(
      [this](frc2::command::Command *command) { LogCommand(command, false); });
  frc2::command::CommandScheduler::GetInstance().onCommandInterrupt.connect(
      [this](frc2::command::Command *command) { LogCommand(command, false); });

  // Reset alert timers
  canInitialErrorTimer.Restart();
  canErrorTimer.Restart();
  canivoreErrorTimer.Restart();
  disabledTimer.Restart();

  // Configure brownout voltage
  frc::RobotController::SetBrownoutVoltage(6.0);

  // Configure DriverStation for sim
  if (Constants::GetRobot() == Constants::Robot::kSimbot) {
    frc::sim::DriverStationSim::SetAllianceStationId(
        frc::AllianceStationID::kBlue1);
    frc::sim::DriverStationSim::NotifyNewData();
  }

  // Create RobotContainer
  robotContainer = new RobotContainer();

  // Switch thread to high priority to improve loop timing
  frc::Threads::SetCurrentThreadPriority(true, 10);
}

void Robot::RobotPeriodic() {
  // Refresh all Phoenix signals
  LoggedTracer::Reset();
  PhoenixUtil::RefreshAll();
  LoggedTracer::Record("PhoenixRefresh");

  // Run virtual subsystems
  VirtualSubsystem::PeriodicAll();

  // Run command scheduler
  frc2::command::CommandScheduler::GetInstance().Run();
  LoggedTracer::Record("Commands");

  // Print auto duration
  if (autonomousCommand != nullptr) {
    if (!autonomousCommand->IsScheduled() && !autoMessagePrinted) {
      if (frc::DriverStation::IsAutonomousEnabled()) {
        printf("*** Auto finished in %.2f secs ***\n",
               frc::Timer::GetFPGATimestamp() - autoStart);
      } else {
        printf("*** Auto cancelled in %.2f secs ***\n",
               frc::Timer::GetFPGATimestamp() - autoStart);
      }
      autoMessagePrinted = true;
    }
  }

  // Robot container periodic methods
  robotContainer->UpdateAlerts();
  robotContainer->UpdateDashboardOutputs();

  // Check CAN status
  auto canStatus = frc::RobotController::GetCANStatus();
  if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
    canErrorTimer.Restart();
  }
  canErrorAlert.Set(!canErrorTimer.HasElapsed(canErrorTimeThreshold) &&
                    !canInitialErrorTimer.HasElapsed(canErrorTimeThreshold));

  // Log CANivore status
  if (Constants::GetMode() == Constants::Mode::kReal) {
    auto canivoreStatus = canivoreReader.GetStatus();
    if (canivoreStatus.has_value()) {
      JUnction::Logger::RecordOutput("CANivoreStatus/Status",
                                     canivoreStatus.value().Status.GetName());
      JUnction::Logger::RecordOutput("CANivoreStatus/Utilization",
                                     canivoreStatus.value().BusUtilization);
      JUnction::Logger::RecordOutput("CANivoreStatus/OffCount",
                                     canivoreStatus.value().BusOffCount);
      JUnction::Logger::RecordOutput("CANivoreStatus/TxFullCount",
                                     canivoreStatus.value().TxFullCount);
      JUnction::Logger::RecordOutput("CANivoreStatus/ReceiveErrorCount",
                                     canivoreStatus.value().REC);
      JUnction::Logger::RecordOutput("CANivoreStatus/TransmitErrorCount",
                                     canivoreStatus.value().TEC);
      if (!canivoreStatus.value().Status.IsOK() ||
          canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
        canivoreErrorTimer.Restart();
      }
    }
    canivoreErrorAlert.Set(
        !canivoreErrorTimer.HasElapsed(canivoreErrorTimeThreshold) &&
        !canInitialErrorTimer.HasElapsed(canErrorTimeThreshold));
  }

  // Log NT client list
  NTClientLogger::Log();

  // Low battery alert
  lowBatteryCycleCount += 1;
  if (frc::DriverStation::IsEnabled()) {
    disabledTimer.Reset();
  }
  if (frc::RobotController::GetBatteryVoltage() <= lowBatteryVoltage &&
      disabledTimer.HasElapsed(lowBatteryDisabledTime) &&
      lowBatteryCycleCount >= lowBatteryMinCycleCount) {
    lowBatteryAlert.Set(true);
    Leds::GetInstance().lowBatteryAlert = true;
  }

  // JIT alert
  jitAlert.Set(IsJITing());

  // Log robot state values
  RobotState::GetInstance().PeriodicLog();

  // Record cycle time
  LoggedTracer::Record("RobotPeriodic");
}

bool Robot::IsJITing() { return frc::Timer::GetFPGATimestamp() < 45.0; }

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {
  autoStart = frc::Timer::GetFPGATimestamp();
  autonomousCommand = robotContainer->GetAutonomousCommand();

  if (autonomousCommand != nullptr) {
    autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  if (autonomousCommand != nullptr) {
    autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

void Robot::LogCommand(frc2::command::Command *command, bool active) {
  std::string name = command->GetName();
  int count = commandCounts[name] + (active ? 1 : -1);
  commandCounts[name] = count;
  JUnction::Logger::RecordOutput(
      "CommandsUnique/" + name + "_" +
          std::to_string(reinterpret_cast<uintptr_t>(command)),
      active);
  JUnction::Logger::RecordOutput("CommandsAll/" + name, count > 0);
}
