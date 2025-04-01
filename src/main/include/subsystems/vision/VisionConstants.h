// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <string>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/math/units/units.h>

#include "Constants.h"
#include "util/LoggedTunableNumber.h"

struct CameraConfig {
  std::function<frc::Pose3d()> pose;
  std::string id;
  int width;
  int height;
  int autoExposure;
  int exposure;
  double gain;
  double stdDevFactor;
};

class VisionConstants {
public:
  static constexpr bool forceEnableInstanceLogging = true;
  static constexpr bool enableInstanceLogging =
      forceEnableInstanceLogging || Constants::GetMode() == Mode::REPLAY;

  static constexpr double ambiguityThreshold = 0.4;
  static constexpr double targetLogTimeSecs = 0.1;
  static constexpr double fieldBorderMargin = 0.5;
  static constexpr double xyStdDevCoefficient = 0.015;
  static constexpr double thetaStdDevCoefficient = 0.03;
  static constexpr double demoTagPosePersistenceSecs = 0.5;
  static constexpr double objDetectConfidenceThreshold = 0.8;
  static LoggedTunableNumber timestampOffset;

  static LoggedTunableNumber *pitchAdjustments;
  static CameraConfig *cameras;

private:
  static int monoExposure;
  static int colorExposure;
  static double monoGain;
  static double colorGain;

  VisionConstants() = delete;
};

// Initialize static members
inline LoggedTunableNumber VisionConstants::timestampOffset =
    LoggedTunableNumber("AprilTagVision/TimestampOffset", 0.0);

inline int VisionConstants::monoExposure = 12000;
inline int VisionConstants::colorExposure = 12000;
inline double VisionConstants::monoGain = 0.3;
inline double VisionConstants::colorGain = 0.3;

inline LoggedTunableNumber *VisionConstants::pitchAdjustments = []() {
  switch (Constants::GetRobot()) {
  case Robot::DEVBOT:
    return new LoggedTunableNumber[2]{
        LoggedTunableNumber("Vision/PitchAdjust0", 0.0),
        LoggedTunableNumber("Vision/PitchAdjust1", 0.0)};
  case Robot::COMPBOT:
    return new LoggedTunableNumber[4]{
        LoggedTunableNumber("Vision/PitchAdjust0", 0.0),
        LoggedTunableNumber("Vision/PitchAdjust1", 0.0),
        LoggedTunableNumber("Vision/PitchAdjust2", 0.0),
        LoggedTunableNumber("Vision/PitchAdjust3", 0.0)};
  default:
    return new LoggedTunableNumber[0];
  }
}();

inline CameraConfig *VisionConstants::cameras = []() {
  switch (Constants::GetRobot()) {
  case Robot::DEVBOT:
    return new CameraConfig[2]{
        {[]() {
           return frc::Pose3d(
               0.254, 0.2032, 0.21113,
               frc::Rotation3d(
                   0.0,
                   units::degree_to_radian(
                       -25.0 + VisionConstants::pitchAdjustments[0].Get()),
                   units::degree_to_radian(-20.0)));
         },
         "40265450", 1600, 1200, 0, VisionConstants::monoExposure,
         VisionConstants::monoGain, 1.0},
        {[]() {
           return frc::Pose3d(
               0.254, -0.2032, 0.21113,
               frc::Rotation3d(
                   0.0,
                   units::degree_to_radian(
                       -25.0 + VisionConstants::pitchAdjustments[1].Get()),
                   units::degree_to_radian(20.0)));
         },
         "40265453", 1600, 1200, 0, VisionConstants::monoExposure,
         VisionConstants::monoGain, 1.0}};
  case Robot::COMPBOT:
    return new CameraConfig[4]{
        {[]() {
           return frc::Pose3d(
               units::inch_to_meter(10), units::inch_to_meter(10),
               units::inch_to_meter(18.5),
               frc::Rotation3d(
                   0.0,
                   units::degree_to_radian(
                       29 + VisionConstants::pitchAdjustments[0].Get()),
                   units::degree_to_radian(-23.31)));
         },
         "40552080", 1600, 1200, 0, VisionConstants::monoExposure,
         VisionConstants::monoGain, 1.0},
        {[]() {
           return frc::Pose3d(
               units::inch_to_meter(10), units::inch_to_meter(-10.0),
               units::inch_to_meter(18.5),
               frc::Rotation3d(
                   0.0,
                   units::degree_to_radian(
                       29 + VisionConstants::pitchAdjustments[1].Get()),
                   units::degree_to_radian(23.31)));
         },
         "40552081", 1600, 1200, 0, VisionConstants::monoExposure,
         VisionConstants::monoGain, 1.0},
        {[]() {
           return frc::Pose3d(
               units::inch_to_meter(3), units::inch_to_meter(10),
               units::inch_to_meter(25),
               frc::Rotation3d(
                   0.0,
                   units::degree_to_radian(
                       -40.0 + VisionConstants::pitchAdjustments[2].Get()),
                   units::degree_to_radian(198.394)));
         },
         "40265453", 1600, 1200, 0, VisionConstants::monoExposure,
         VisionConstants::monoGain, 1.0},
        {[]() {
           return frc::Pose3d(
               units::inch_to_meter(9), units::inch_to_meter(10),
               units::inch_to_meter(25),
               frc::Rotation3d(0.0,
                               units::degree_to_radian(
                                   VisionConstants::pitchAdjustments[3].Get()),
                               units::degree_to_radian(-15)));
         },
         "24737133", 1280, 960, 0, VisionConstants::colorExposure,
         VisionConstants::colorGain, 1.25}};
  default:
    return new CameraConfig[0];
  }
}();