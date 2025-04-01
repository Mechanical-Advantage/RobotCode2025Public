// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/superstructure/SuperstructurePose.h"

#include <cmath>
#include <map>

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/math/units/units.h>

#include "Constants.h"
#include "FieldConstants.h"
#include "RobotState.h"
#include "subsystems/superstructure/SuperstructureConstants.h"
#include "subsystems/superstructure/dispenser/Dispenser.h"
#include "util/GeomUtil.h"
#include "util/LoggedTunableNumber.h"

LoggedTunableNumber SuperstructurePose::intakeHeightBaseline =
    LoggedTunableNumber("Superstructure/Intake/ElevatorBaseline", 0.04);
LoggedTunableNumber SuperstructurePose::intakeHeightRange =
    LoggedTunableNumber("Superstructure/Intake/ElevatorRange", 0.008);
LoggedTunableNumber SuperstructurePose::intakeHeightTimeFactor =
    LoggedTunableNumber("Superstructure/Intake/ElevatorTimeFactor", 25.0);

std::map<FieldConstants::ReefLevel, LoggedTunableNumber>
    SuperstructurePose::ejectDistance;
std::map<FieldConstants::ReefLevel, LoggedTunableNumber>
    SuperstructurePose::ejectAngles;
std::map<FieldConstants::ReefLevel, LoggedTunableNumber>
    SuperstructurePose::heightFudges;

void SuperstructurePose::AddInitialValue(
    std::map<FieldConstants::ReefLevel, LoggedTunableNumber> &map,
    FieldConstants::ReefLevel reefLevel, double initialValue,
    const std::string &key) {
  map[reefLevel] = LoggedTunableNumber("Superstructure/ReefScore/" + key + "/" +
                                           static_cast<int>(reefLevel),
                                       initialValue);
}

SuperstructurePose::SuperstructurePose(
    std::function<double()> elevatorHeight,
    std::function<frc::Rotation2d()> pivotAngle)
    : elevatorHeight(elevatorHeight), pivotAngle(pivotAngle) {}

SuperstructurePose::SuperstructurePose()
    : elevatorHeight([]() { return 0.0; }),
      pivotAngle([]() { return frc::Rotation2d::Radians(0.0); }) {}

SuperstructurePose::DispenserPose::DispenserPose(Preset preset) {
  switch (preset) {
  case Preset::L1:
    pose = GetCoralScorePose(FieldConstants::ReefLevel::L1);
    break;
  case Preset::L2:
    pose = GetCoralScorePose(FieldConstants::ReefLevel::L2);
    break;
  case Preset::L3:
    pose = GetCoralScorePose(FieldConstants::ReefLevel::L3);
    break;
  case Preset::L4:
    pose = GetCoralScorePose(FieldConstants::ReefLevel::L4);
    break;
  }
}

double SuperstructurePose::DispenserPose::GetElevatorHeight() {
  return (pose.Y() - SuperstructureConstants::dispenserOrigin2d.Y()) /
         SuperstructureConstants::elevatorAngle.Sin();
}

double SuperstructurePose::DispenserPose::GetDispenserAngleDeg() {
  return pose.Rotation().Degrees();
}

frc::Transform2d SuperstructurePose::DispenserPose::ToRobotPose() {
  return frc::Transform2d(
      GetElevatorHeight() * SuperstructureConstants::elevatorAngle.Cos() +
          pose.X() + SuperstructureConstants::dispenserOrigin2d.X(),
      0.0, frc::Rotation2d::Radians(M_PI));
}

SuperstructurePose::DispenserPose
SuperstructurePose::DispenserPose::ForCoralScore(
    FieldConstants::ReefLevel reefLevel) {
  switch (reefLevel) {
  case FieldConstants::ReefLevel::L1:
    return DispenserPose::Preset::L1;
  case FieldConstants::ReefLevel::L2:
    return DispenserPose::Preset::L2;
  case FieldConstants::ReefLevel::L3:
    return DispenserPose::Preset::L3;
  case FieldConstants::ReefLevel::L4:
    return DispenserPose::Preset::L4;
  }
  return DispenserPose::Preset::L1;
}

frc::Pose2d SuperstructurePose::DispenserPose::GetCoralScorePose(
    FieldConstants::ReefLevel reefLevel) {
  double dispenserAngleDeg = ejectAngles[reefLevel].Get();
  if (Constants::GetRobot() == RobotType::DEVBOT) {
    dispenserAngleDeg = -30.0;
  }
  return frc::Pose2d(
      frc::Pose2d(0.0, reefLevel.height + heightFudges[reefLevel].Get(),
                  frc::Rotation2d::Degrees(-dispenserAngleDeg))
          .TransformBy(GeomUtil::ToTransform2d(
              ejectDistance[reefLevel].Get() +
                  SuperstructureConstants::pivotToTunnelFront,
              0.0))
          .Translation(),
      frc::Rotation2d::Degrees(dispenserAngleDeg));
}

SuperstructurePose::SuperstructurePose(Preset preset) {
  switch (preset) {
  case Preset::STOW:
    *this = CreateFromPreset(preset);
    break;
  case Preset::INTAKE:
    *this = CreateFromPreset(preset);
    break;
  case Preset::GOODBYE_CORAL:
    *this = CreateFromPreset(preset);
    break;
  case Preset::L1:
    *this = CreateFromPreset(preset);
    break;
  case Preset::L1_EJECT:
    *this = CreateFromPreset(preset);
    break;
  case Preset::L2:
    *this = CreateFromPreset(preset);
    break;
  case Preset::L3:
    *this = CreateFromPreset(preset);
    break;
  case Preset::L4:
    *this = CreateFromPreset(preset);
    break;
  case Preset::ALGAE_L2_INTAKE:
    *this = CreateFromPreset(preset);
    break;
  case Preset::ALGAE_L3_INTAKE:
    *this = CreateFromPreset(preset);
    break;
  case Preset::PRE_THROW:
    *this = CreateFromPreset(preset);
    break;
  case Preset::THROW:
    *this = CreateFromPreset(preset);
    break;
  case Preset::ALGAE_STOW:
    *this = CreateFromPreset(preset);
    break;
  }
}

SuperstructurePose SuperstructurePose::CreateFromPreset(Preset preset) {
  switch (preset) {
  case Preset::STOW:
    return SuperstructurePose(intakeHeightBaseline.Get(),
                              frc::Rotation2d::Degrees(-18.0));
  case Preset::INTAKE:
    return SuperstructurePose(
        [&]() {
          return intakeHeightBaseline.Get() +
                 intakeHeightRange.Get() *
                     std::sin(frc::Timer::GetFPGATimestamp() *
                              intakeHeightTimeFactor.Get());
        },
        LoggedTunableNumber("Superstructure/Intake/IntakePivot", -18.0).Get());
  case Preset::GOODBYE_CORAL:
    return SuperstructurePose(
        intakeHeightBaseline,
        LoggedTunableNumber("Superstructure/Intake/EjectPivot", 18.0).Get());
  case Preset::L1:
  case Preset::L1_EJECT: {
    DispenserPose dispenserPose =
        DispenserPose::ForCoralScore(FieldConstants::ReefLevel::L1);
    LoggedTunableNumber l1LaunchAdjustment =
        (preset == Preset::L1_EJECT)
            ? LoggedTunableNumber("Superstructure/ReefScore/L1LaunchAdjustment",
                                  0.0)
            : LoggedTunableNumber("Superstructure/ReefScore/L1LaunchAdjustment",
                                  0.0);
    return SuperstructurePose(
        [&]() {
          return dispenserPose.GetElevatorHeight() + l1LaunchAdjustment.Get();
        },
        [&]() {
          if (RobotState::GetInstance().GetDistanceToBranch().has_value() &&
              preset != Preset::L1) {
            return frc::Rotation2d::Degrees(
                frc::Rotation2d(
                    RobotState::GetInstance().GetDistanceToBranch().value(),
                    FieldConstants::ReefLevel::L1.height -
                        (dispenserPose.GetElevatorHeight() *
                             SuperstructureConstants::elevatorAngle.Sin() +
                         SuperstructureConstants::dispenserOrigin2d.Y()))
                    .Degrees());
          } else {
            return frc::Rotation2d::Degrees(
                dispenserPose.GetDispenserAngleDeg());
          }
        });
  }
  case Preset::L2: {
    DispenserPose dispenserPose =
        DispenserPose::ForCoralScore(FieldConstants::ReefLevel::L2);
    return SuperstructurePose(
        [&]() { return dispenserPose.GetElevatorHeight(); },
        [&]() {
          if (RobotState::GetInstance().GetDistanceToBranch().has_value()) {
            return frc::Rotation2d::Degrees(
                frc::Rotation2d(
                    RobotState::GetInstance().GetDistanceToBranch().value(),
                    FieldConstants::ReefLevel::L2.height -
                        (dispenserPose.GetElevatorHeight() *
                             SuperstructureConstants::elevatorAngle.Sin() +
                         SuperstructureConstants::dispenserOrigin2d.Y()))
                    .Degrees());
          } else {
            return frc::Rotation2d::Degrees(
                dispenserPose.GetDispenserAngleDeg());
          }
        });
  }
  case Preset::L3: {
    DispenserPose dispenserPose =
        DispenserPose::ForCoralScore(FieldConstants::ReefLevel::L3);
    return SuperstructurePose(
        [&]() { return dispenserPose.GetElevatorHeight(); },
        [&]() {
          if (RobotState::GetInstance().GetDistanceToBranch().has_value()) {
            return frc::Rotation2d::Degrees(
                frc::Rotation2d(
                    RobotState::GetInstance().GetDistanceToBranch().value(),
                    FieldConstants::ReefLevel::L3.height -
                        (dispenserPose.GetElevatorHeight() *
                             SuperstructureConstants::elevatorAngle.Sin() +
                         SuperstructureConstants::dispenserOrigin2d.Y()))
                    .Degrees());
          } else {
            return frc::Rotation2d::Degrees(
                dispenserPose.GetDispenserAngleDeg());
          }
        });
  }
  case Preset::L4: {
    DispenserPose dispenserPose =
        DispenserPose::ForCoralScore(FieldConstants::ReefLevel::L4);
    return SuperstructurePose(
        [&]() { return dispenserPose.GetElevatorHeight(); },
        [&]() {
          if (RobotState::GetInstance().GetDistanceToBranch().has_value()) {
            return frc::Rotation2d::Degrees(
                frc::Rotation2d(
                    RobotState::GetInstance().GetDistanceToBranch().value(),
                    FieldConstants::ReefLevel::L4.height -
                        (dispenserPose.GetElevatorHeight() *
                             SuperstructureConstants::elevatorAngle.Sin() +
                         SuperstructureConstants::dispenserOrigin2d.Y()))
                    .Degrees());
          } else {
            return frc::Rotation2d::Degrees(
                dispenserPose.GetDispenserAngleDeg());
          }
        });
  }
  case Preset::ALGAE_L2_INTAKE:
    return SuperstructurePose(1.5, 0.0);
  case Preset::ALGAE_L3_INTAKE:
    return SuperstructurePose(1.5, 0.0);
  case Preset::PRE_THROW:
    return SuperstructurePose(1.0, Dispenser::maxAngle.Degrees());
  case Preset::THROW:
    return SuperstructurePose(SuperstructureConstants::elevatorMaxTravel,
                              Dispenser::maxAngle.Degrees());
  case Preset::ALGAE_STOW:
    return SuperstructurePose(0.15, 0.0);
  }
  return SuperstructurePose();
}

static bool initialized = false;

void InitializeSuperstructurePose() {
  if (!initialized) {
    SuperstructurePose::AddInitialValue(
        SuperstructurePose::ejectDistance, FieldConstants::ReefLevel::L1,
        units::inch_to_meter(2.0), "EjectDistance");
    SuperstructurePose::AddInitialValue(
        SuperstructurePose::ejectDistance, FieldConstants::ReefLevel::L2,
        units::inch_to_meter(8.0), "EjectDistance");
    SuperstructurePose::AddInitialValue(
        SuperstructurePose::ejectDistance, FieldConstants::ReefLevel::L3,
        units::inch_to_meter(5.5), "EjectDistance");
    SuperstructurePose::AddInitialValue(
        SuperstructurePose::ejectDistance, FieldConstants::ReefLevel::L4,
        units::inch_to_meter(6.0), "EjectDistance");

    SuperstructurePose::AddInitialValue(SuperstructurePose::ejectAngles,
                                        FieldConstants::ReefLevel::L1, 20.0,
                                        "EjectAngles");
    SuperstructurePose::AddInitialValue(SuperstructurePose::ejectAngles,
                                        FieldConstants::ReefLevel::L2, -20.0,
                                        "EjectAngles");
    SuperstructurePose::AddInitialValue(SuperstructurePose::ejectAngles,
                                        FieldConstants::ReefLevel::L3, -35.0,
                                        "EjectAngles");
    SuperstructurePose::AddInitialValue(SuperstructurePose::ejectAngles,
                                        FieldConstants::ReefLevel::L4, -48.0,
                                        "EjectAngles");

    SuperstructurePose::AddInitialValue(SuperstructurePose::heightFudges,
                                        FieldConstants::ReefLevel::L1, 0.08,
                                        "HeightFudges");
    SuperstructurePose::AddInitialValue(
        SuperstructurePose::heightFudges, FieldConstants::ReefLevel::L2,
        units::inch_to_meter(0.0), "HeightFudges");
    SuperstructurePose::AddInitialValue(
        SuperstructurePose::heightFudges, FieldConstants::ReefLevel::L3,
        units::inch_to_meter(2.5), "HeightFudges");
    SuperstructurePose::AddInitialValue(
        SuperstructurePose::heightFudges, FieldConstants::ReefLevel::L4,
        units::inch_to_meter(1.0), "HeightFudges");

    initialized = true;
  }
}

// Ensure the static initialization happens before any SuperstructurePose is
// constructed
struct SuperstructurePoseInitializer {
  SuperstructurePoseInitializer() { InitializeSuperstructurePose(); }
};

static SuperstructurePoseInitializer initializer;
