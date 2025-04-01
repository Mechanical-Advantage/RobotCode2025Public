// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <map>
#include <vector>

#include <frc/Alert.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/math/VecBuilder.h>

#include "FieldConstants.h"
#include "RobotState.h"
#include "subsystems/leds/Leds.h"
#include "subsystems/vision/VisionConstants.h"
#include "subsystems/vision/VisionIO.h"
#include "thirdparty/junction/Logger.h"
#include "thirdparty/junction/networktables/LoggedNetworkBoolean.h"
#include "util/GeomUtil.h"
#include "util/LoggedTracer.h"
#include "util/VirtualSubsystem.h"

class Vision : public VirtualSubsystem {
public:
  Vision(std::function<FieldConstants::AprilTagLayoutType()>
             aprilTagLayoutSupplier,
         VisionIO *io[], int ioCount);

  void Periodic() override;

private:
  std::function<FieldConstants::AprilTagLayoutType()> aprilTagLayoutSupplier;
  VisionIO **io;
  VisionIOInputsAutoLogged *inputs;
  AprilTagVisionIOInputsAutoLogged *aprilTagInputs;
  ObjDetectVisionIOInputsAutoLogged *objDetectInputs;
  int ioCount;
  LoggedNetworkBoolean recordingRequest =
      LoggedNetworkBoolean("/SmartDashboard/Enable Recording", false);

  std::map<int, double> lastFrameTimes;
  std::map<int, double> lastTagDetectionTimes;

  static constexpr double disconnectedTimeout = 0.5;
  frc::Timer *disconnectedTimers;
  frc::Alert *disconnectedAlerts;
};