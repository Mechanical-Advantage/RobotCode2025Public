// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>

#include <frc/Alert.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>

#include "Robot.h"
#include "thirdparty/junction/AutoLog.h"
#include "thirdparty/junction/Logger.h"
#include "util/gslam/GenericSlamElevatorIO.h"

template <typename G> class GenericSlamElevator {
public:
  struct SlamElevatorGoal {
    virtual std::function<double()> GetSlammingCurrent() = 0;
    virtual bool IsStopAtGoal() = 0;
    virtual SlamElevatorState GetState() = 0;
    virtual ~SlamElevatorGoal() = default;
  };

  enum class SlamElevatorState { IDLING, RETRACTING, EXTENDING };

  GenericSlamElevator(const std::string &name, GenericSlamElevatorIO &io,
                      double staticTimeSecs, double minVelocityThresh);

  void SetCoastOverride(std::function<bool()> coastOverride);
  void Periodic();

  JUnction_AUTO_LOG_VARS() bool Slammed();
  JUnction_AUTO_LOG_VARS() bool Extended();
  JUnction_AUTO_LOG_VARS() bool Retracted();

protected:
  virtual G *GetGoal() = 0;

  GenericSlamElevatorIO &io;
  GenericSlamElevatorIOInputsAutoLogged inputs;

private:
  void SetBrakeMode(bool enable);

  std::string name;
  double staticTimeSecs;
  double minVelocityThresh;

  G *lastGoal = nullptr;

  bool slammed = false;
  frc::Timer staticTimer;

  bool brakeModeEnabled = false;
  std::function<bool()> coastModeSupplier = []() { return false; };

  frc::Alert disconnected;
};