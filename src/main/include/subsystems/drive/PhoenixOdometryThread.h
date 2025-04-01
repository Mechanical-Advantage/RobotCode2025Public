// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <vector>

#include "ctre/phoenix6/BaseStatusSignal.hpp"
#include "ctre/phoenix6/CANBus.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"

#include "frc/RobotController.h"
#include "frc/Threads.h"

#include "org/littletonrobotics/frc2025/subsystems/drive/Drive.h"
#include "org/littletonrobotics/frc2025/subsystems/drive/DriveConstants.h"

class PhoenixOdometryThread {
public:
  static PhoenixOdometryThread &GetInstance();

  std::queue<double> *RegisterSignal(
      ctre::phoenix6::signals::StatusSignal<units::radian_t> &signal);
  std::queue<double> *RegisterSignal(std::function<double()> signal);
  std::queue<double> *MakeTimestampQueue();

  void Run();

private:
  PhoenixOdometryThread();
  ~PhoenixOdometryThread() = default;

  std::mutex signalsMutex;
  std::vector<ctre::phoenix6::BaseStatusSignal *> phoenixSignals;
  std::vector<std::function<double()>> genericSignals;
  std::vector<std::queue<double> *> phoenixQueues;
  std::vector<std::queue<double> *> genericQueues;
  std::vector<std::queue<double> *> timestampQueues;

  static bool isCANFD;
  static PhoenixOdometryThread instance;
};