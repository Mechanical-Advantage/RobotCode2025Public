// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/gslam/GenericSlamElevator.h"

#include <cmath>

#include <frc/Alert.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>

#include "Robot.h"
#include "thirdparty/junction/Logger.h"

template <typename G>
GenericSlamElevator<G>::GenericSlamElevator(const std::string &name,
                                            GenericSlamElevatorIO &io,
                                            double staticTimeSecs,
                                            double minVelocityThresh)
    : io(io), name(name), staticTimeSecs(staticTimeSecs),
      minVelocityThresh(minVelocityThresh),
      disconnected(name + " disconnected!", frc::Alert::AlertType::kWarning) {
  SetBrakeMode(true);
}

template <typename G>
void GenericSlamElevator<G>::SetCoastOverride(
    std::function<bool()> coastOverride) {
  coastModeSupplier = coastOverride;
}

template <typename G> void GenericSlamElevator<G>::SetBrakeMode(bool enable) {
  if (brakeModeEnabled == enable)
    return;
  brakeModeEnabled = enable;
  io.SetBrakeMode(brakeModeEnabled);
}

template <typename G> void GenericSlamElevator<G>::Periodic() {
  io.UpdateInputs(inputs);
  Logger::ProcessInputs(name, inputs);

  if (frc::DriverStation::IsEnabled()) {
    SetBrakeMode(true);
  }

  if (lastGoal != nullptr && GetGoal() != lastGoal) {
    slammed = false;
    staticTimer.Stop();
    staticTimer.Reset();
  }
  lastGoal = GetGoal();

  disconnected.Set(!inputs.data.motorConnected && !Robot::IsJITing());

  if (!slammed) {
    if (std::abs(inputs.data.velocityRadsPerSec) <= minVelocityThresh) {
      staticTimer.Start();
    } else {
      staticTimer.Stop();
      staticTimer.Reset();
    }
    slammed = staticTimer.HasElapsed(staticTimeSecs) ||
              frc::DriverStation::IsAutonomousEnabled();
  } else {
    staticTimer.Stop();
    staticTimer.Reset();
  }

  if (!slammed) {
    io.RunCurrent(GetGoal()->GetSlammingCurrent()());
  } else {
    if (GetGoal()->IsStopAtGoal()) {
      io.Stop();
    } else {
      io.RunCurrent(GetGoal()->GetSlammingCurrent()());
    }
  }

  if (frc::DriverStation::IsDisabled()) {
    io.Stop();
    lastGoal = nullptr;
    staticTimer.Stop();
    staticTimer.Reset();
    if (std::abs(inputs.data.velocityRadsPerSec) > minVelocityThresh) {
      slammed = false;
    }
  }

  SetBrakeMode(!coastModeSupplier());

  Logger::RecordOutput("Chariot/Goal", GetGoal()->ToString());
  Logger::RecordOutput("Chariot/BrakeModeEnabled", brakeModeEnabled);
}

template <typename G> bool GenericSlamElevator<G>::Slammed() { return slammed; }

template <typename G> bool GenericSlamElevator<G>::Extended() {
  return GetGoal()->GetState() == SlamElevatorState::EXTENDING && slammed;
}

template <typename G> bool GenericSlamElevator<G>::Retracted() {
  return GetGoal()->GetState() == SlamElevatorState::RETRACTING && slammed;
}