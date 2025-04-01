// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/superstructure/SuperstructureVisualizer.h"

#include <cmath>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/math/units/units.h>
#include <frc/util/Color.h>
#include <frc/util/Color8Bit.h>

#include "Constants.h"
#include "FieldConstants.h"
#include "RobotState.h"
#include "subsystems/superstructure/SuperstructureConstants.h"
#include "thirdparty/junction/Logger.h"
#include "thirdparty/junction/mechanism/LoggedMechanism2d.h"
#include "thirdparty/junction/mechanism/LoggedMechanismLigament2d.h"
#include "thirdparty/junction/mechanism/LoggedMechanismRoot2d.h"
#include "util/EqualsUtil.h"

SuperstructureVisualizer::SuperstructureVisualizer(const std::string &name)
    : name(name),
      mechanism(units::inch_to_meter(28.0), units::feet_to_meter(7.0),
                frc::Color8Bit(frc::Color::kDarkGray)) {
  LoggedMechanismRoot2d root = mechanism.GetRoot(
      name + " Root", SuperstructureConstants::superstructureOrigin2d.X(),
      SuperstructureConstants::superstructureOrigin2d.Y());
  elevatorMechanism = root.Append(LoggedMechanismLigament2d(
      name + " Elevator", units::inch_to_meter(26.0),
      SuperstructureConstants::elevatorAngle.Degrees(), 4.0,
      frc::Color8Bit(frc::Color::kFirstBlue)));
  pivotMechanism = elevatorMechanism.Append(
      LoggedMechanismLigament2d(name + " Pivot", units::inch_to_meter(6.0), 0.0,
                                8.0, frc::Color8Bit(frc::Color::kFirstRed)));
}

void SuperstructureVisualizer::Update(double elevatorHeightMeters,
                                      frc::Rotation2d pivotFinalAngle,
                                      double algaeIntakePosition,
                                      bool hasAlgae) {
  if (Constants::GetMode() != Mode::REAL) {
    elevatorMechanism.SetLength(
        EqualsUtil::EpsilonEquals(elevatorHeightMeters, 0.0)
            ? units::inch_to_meter(1.0)
            : elevatorHeightMeters);
    pivotMechanism.SetAngle(pivotFinalAngle -
                            SuperstructureConstants::elevatorAngle);
    Logger::RecordOutput("Mechanism2d/" + name, mechanism);
  }

  // Max of top of carriage or starting height
  const double heightFromBottom = elevatorHeightMeters +
                                  SuperstructureConstants::dispenserToBottom +
                                  SuperstructureConstants::dispenserToTop +
                                  SuperstructureConstants::stageThickness * 2.0;
  const double firstStageHeight =
      std::max(heightFromBottom - SuperstructureConstants::firstStageHeight -
                   SuperstructureConstants::stageThickness,
               SuperstructureConstants::stageThickness);
  const double secondStageHeight =
      std::max(firstStageHeight - SuperstructureConstants::stageHeight +
                   SuperstructureConstants::stageToStage -
                   SuperstructureConstants::stageThickness,
               0.0);

  frc::Pose3d pivotPose3d = frc::Pose3d(
      SuperstructureVisualizer::intakeOrigin3d +
          frc::Translation3d(
              elevatorHeightMeters,
              frc::Rotation3d(
                  0.0, -SuperstructureConstants::elevatorAngle.Radians(), 0.0)),
      frc::Rotation3d(
          0.0,
          -(pivotFinalAngle.Radians() +
            units::degree_to_radian(
                SuperstructureVisualizer::modelToRealDispenserRotation)),
          0.0));

  Logger::RecordOutput(
      "Mechanism3d/" + name + "/Superstructure",
      frc::Pose3d(
          SuperstructureConstants::superstructureOrigin3d +
              frc::Translation3d(
                  secondStageHeight,
                  frc::Rotation3d(
                      0.0, -SuperstructureConstants::elevatorAngle.Radians(),
                      0.0)),
          frc::Rotation3d::Identity()),
      frc::Pose3d(
          SuperstructureConstants::superstructureOrigin3d +
              frc::Translation3d(
                  firstStageHeight,
                  frc::Rotation3d(
                      0.0, -SuperstructureConstants::elevatorAngle.Radians(),
                      0.0)),
          frc::Rotation3d::Identity()),
      frc::Pose3d(pivotPose3d.Translation(), frc::Rotation3d::Identity()),
      pivotPose3d,
      frc::Pose3d(SuperstructureVisualizer::intakeOrigin3d +
                      frc::Translation3d(
                          algaeIntakePosition,
                          frc::Rotation3d(
                              0.0,
                              units::degree_to_radian(
                                  -SuperstructureVisualizer::intakeAngleDeg),
                              0.0)),
                  frc::Rotation3d::Identity()));

  if (hasAlgae) {
    Logger::RecordOutput(
        "Mechanism3d/" + name + "/Algae",
        (RobotState::GetInstance().GetEstimatedPose() +
         frc::Transform3d(frc::Pose3d::Identity(), pivotPose3d) +
         frc::Transform3d(SuperstructureConstants::pivotToTunnelFront +
                              FieldConstants::algaeDiameter / 2.0,
                          0.0, 0.0, frc::Rotation3d::Identity()))
            .Translation());
  } else {
    Logger::RecordOutput("Mechanism3d/" + name + "/Algae",
                         std::vector<frc::Translation3d>{});
  }
}

void SuperstructureVisualizer::UpdateSimIntake(double angleRad) {
  Logger::RecordOutput(
      "Mechanism3d/AlgaeIntakeSim",
      frc::Pose3d(SuperstructureVisualizer::intakeOrigin3d,
                  frc::Rotation3d(0.0, M_PI / 2.0 - angleRad, 0.0)));
}