// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "Constants.h"

#include <iostream>

#include <frc/Alert.h>
#include <frc/RobotBase.h>

Constants::Robot Constants::robotType = Constants::Robot::kCompbot;
bool Constants::disableHAL = false;

Constants::Robot Constants::GetRobot() {
  if (!disableHAL && frc::RobotBase::IsReal() && robotType == Robot::kSimbot) {
    frc::Alert("Invalid robot selected, using competition robot as default.",
               frc::Alert::AlertType::kError)
        .Set(true);
    robotType = Robot::kCompbot;
  }
  return robotType;
}

Constants::Mode Constants::GetMode() {
  switch (robotType) {
  case Robot::kDevbot:
  case Robot::kCompbot:
    return frc::RobotBase::IsReal() ? Mode::kReal : Mode::kReplay;
  case Robot::kSimbot:
    return Mode::kSim;
  default:
    return Mode::kReal;
  }
}

void Constants::DisableHAL() { Constants::disableHAL = true; }

void Constants::CheckDeploy::MainFunction() {
  if (Constants::robotType == Constants::Robot::kSimbot) {
    std::cerr << "Cannot deploy, invalid robot selected: "
              << static_cast<int>(Constants::robotType) << std::endl;
    std::exit(1);
  }
}

void Constants::CheckPullRequest::MainFunction() {
  if (Constants::robotType != Constants::Robot::kCompbot ||
      Constants::tuningMode) {
    std::cerr << "Do not merge, non-default constants are configured."
              << std::endl;
    std::exit(1);
  }
}