// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <string>

#include <frc/Alert.h>
#include <frc/RobotBase.h>

class Constants {
public:
  static constexpr double loopPeriodSecs = 0.02;
  static constexpr bool tuningMode = false;

  enum class Robot { kSimbot, kDevbot, kCompbot };
  enum class Mode { kReal, kSim, kReplay };

  static Robot GetRobot();
  static Mode GetMode();

  static bool disableHAL;

  static void DisableHAL();

  class CheckDeploy {
  public:
    static void MainFunction();
  };

  class CheckPullRequest {
  public:
    static void MainFunction();
  };

private:
  static Robot robotType;

  Constants() = delete;
};