// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/vision/Vision.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <vector>

#include <frc/Alert.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/math/VecBuilder.h>

#include "FieldConstants.h"
#include "RobotState.h"
#include "subsystems/leds/Leds.h"
#include "subsystems/vision/VisionConstants.h"
#include "thirdparty/junction/Logger.h"
#include "util/GeomUtil.h"
#include "util/LoggedTracer.h"

Vision::Vision(
    std::function<FieldConstants::AprilTagLayoutType()> aprilTagLayoutSupplier,
    VisionIO *io[], int ioCount)
    : aprilTagLayoutSupplier(aprilTagLayoutSupplier), io(io),
      inputs(new VisionIOInputsAutoLogged[ioCount]),
      aprilTagInputs(new AprilTagVisionIOInputsAutoLogged[ioCount]),
      objDetectInputs(new ObjDetectVisionIOInputsAutoLogged[ioCount]),
      ioCount(ioCount), disconnectedTimers(new frc::Timer[ioCount]),
      disconnectedAlerts(new frc::Alert[ioCount]) {
  for (int i = 0; i < ioCount; i++) {
    disconnectedAlerts[i] = frc::Alert("", frc::Alert::AlertType::kError);
    lastFrameTimes[i] = 0.0;
    disconnectedTimers[i].Start();
  }
}

void Vision::Periodic() {
  for (int i = 0; i < ioCount; i++) {
    io[i]->UpdateInputs(inputs[i], aprilTagInputs[i], objDetectInputs[i]);
    Logger::ProcessInputs("Vision/Inst" + std::to_string(i), inputs[i]);
    Logger::ProcessInputs("AprilTagVision/Inst" + std::to_string(i),
                          aprilTagInputs[i]);
    Logger::ProcessInputs("ObjDetectVision/Inst" + std::to_string(i),
                          objDetectInputs[i]);
  }

  bool shouldRecord =
      frc::DriverStation::IsFMSAttached() || recordingRequest.Get();
  for (int i = 0; i < ioCount; i++) {
    io[i]->SetRecording(shouldRecord);
  }

  bool anyNTDisconnected = false;
  for (int i = 0; i < ioCount; i++) {
    if (aprilTagInputs[i].timestamps.size() > 0 ||
        objDetectInputs[i].timestamps.size() > 0) {
      disconnectedTimers[i].Reset();
    }
    bool disconnected = disconnectedTimers[i].HasElapsed(disconnectedTimeout) ||
                        !inputs[i].ntConnected;
    if (disconnected) {
      disconnectedAlerts[i].SetText(
          inputs[i].ntConnected
              ? "Northstar " + std::to_string(i) +
                    " connected to NT but not publishing frames"
              : "Northstar " + std::to_string(i) + " disconnected from NT");
    }
    disconnectedAlerts[i].Set(disconnected);
    anyNTDisconnected = anyNTDisconnected || !inputs[i].ntConnected;
  }
  Leds::GetInstance().visionDisconnected = anyNTDisconnected;

  std::vector<frc::Pose2d> allRobotPoses;
  std::vector<RobotState::VisionObservation> allVisionObservations;
  std::map<int, RobotState::TxTyObservation> allTxTyObservations;
  std::vector<RobotState::AlgaeTxTyObservation> allAlgaeTxTyObservations;

  for (int instanceIndex = 0; instanceIndex < ioCount; instanceIndex++) {
    for (size_t frameIndex = 0;
         frameIndex < aprilTagInputs[instanceIndex].timestamps.size();
         frameIndex++) {
      lastFrameTimes[instanceIndex] = frc::Timer::GetFPGATimestamp();
      double timestamp = aprilTagInputs[instanceIndex].timestamps[frameIndex] +
                         VisionConstants::timestampOffset.Get();
      std::vector<double> values =
          aprilTagInputs[instanceIndex].frames[frameIndex];

      if (values.empty() || values[0] == 0) {
        continue;
      }

      frc::Pose3d cameraPose;
      frc::Pose2d robotPose;
      bool useVisionRotation = false;

      switch (static_cast<int>(values[0])) {
      case 1: {
        cameraPose =
            frc::Pose3d(values[2], values[3], values[4],
                        frc::Rotation3d(frc::Quaternion(values[5], values[6],
                                                        values[7], values[8])));
        robotPose = cameraPose.ToPose2d().TransformBy(
            VisionConstants::cameras[instanceIndex]
                .pose.Get()
                .ToPose2d()
                .ToTransform2d()
                .Inverse());
        useVisionRotation = true;
        break;
      }
      case 2: {
        double error0 = values[1];
        double error1 = values[9];
        frc::Pose3d cameraPose0 =
            frc::Pose3d(values[2], values[3], values[4],
                        frc::Rotation3d(frc::Quaternion(values[5], values[6],
                                                        values[7], values[8])));
        frc::Pose3d cameraPose1 =
            frc::Pose3d(values[10], values[11], values[12],
                        frc::Rotation3d(frc::Quaternion(
                            values[13], values[14], values[15], values[16])));
        frc::Transform2d cameraToRobot = VisionConstants::cameras[instanceIndex]
                                             .pose.Get()
                                             .ToPose2d()
                                             .ToTransform2d()
                                             .Inverse();
        frc::Pose2d robotPose0 =
            cameraPose0.ToPose2d().TransformBy(cameraToRobot);
        frc::Pose2d robotPose1 =
            cameraPose1.ToPose2d().TransformBy(cameraToRobot);

        if (error0 < error1 * VisionConstants::ambiguityThreshold ||
            error1 < error0 * VisionConstants::ambiguityThreshold) {
          frc::Rotation2d currentRotation =
              RobotState::GetInstance().GetRotation();
          frc::Rotation2d visionRotation0 = robotPose0.Rotation();
          frc::Rotation2d visionRotation1 = robotPose1.Rotation();
          if (std::abs((currentRotation - visionRotation0).Radians()) <
              std::abs((currentRotation - visionRotation1).Radians())) {
            cameraPose = cameraPose0;
            robotPose = robotPose0;
          } else {
            cameraPose = cameraPose1;
            robotPose = robotPose1;
          }
        }
        break;
      }
      default:
        continue;
      }

      if (!cameraPose.IsValid() || !robotPose.IsValid()) {
        continue;
      }

      if (robotPose.X() < -VisionConstants::fieldBorderMargin ||
          robotPose.X() > FieldConstants::fieldLength +
                              VisionConstants::fieldBorderMargin ||
          robotPose.Y() < -VisionConstants::fieldBorderMargin ||
          robotPose.Y() >
              FieldConstants::fieldWidth + VisionConstants::fieldBorderMargin) {
        continue;
      }

      std::vector<frc::Pose3d> tagPoses;
      for (size_t i = (values[0] == 1 ? 9 : 17); i < values.size(); i += 10) {
        int tagId = static_cast<int>(values[i]);
        lastTagDetectionTimes[tagId] = frc::Timer::GetFPGATimestamp();
        std::optional<frc::Pose3d> tagPose =
            aprilTagLayoutSupplier().GetLayout().GetTagPose(
                static_cast<int>(values[i]));
        if (tagPose.has_value()) {
          tagPoses.push_back(tagPose.value());
        }
      }
      if (tagPoses.empty())
        continue;

      double totalDistance = 0.0;
      for (const frc::Pose3d &tagPose : tagPoses) {
        totalDistance +=
            tagPose.Translation().Distance(cameraPose.Translation());
      }
      double avgDistance = totalDistance / tagPoses.size();

      double xyStdDev = VisionConstants::xyStdDevCoefficient *
                        std::pow(avgDistance, 2.0) / tagPoses.size() *
                        VisionConstants::cameras[instanceIndex].stdDevFactor;
      double thetaStdDev =
          useVisionRotation
              ? VisionConstants::thetaStdDevCoefficient *
                    std::pow(avgDistance, 2.0) / tagPoses.size() *
                    VisionConstants::cameras[instanceIndex].stdDevFactor
              : std::numeric_limits<double>::infinity();
      allVisionObservations.emplace_back(
          robotPose, timestamp,
          frc::VecBuilder<3>::Create(xyStdDev, xyStdDev, thetaStdDev));
      allRobotPoses.push_back(robotPose);

      if (VisionConstants::enableInstanceLogging) {
        Logger::RecordOutput("AprilTagVision/Inst" +
                                 std::to_string(instanceIndex) + "/LatencySecs",
                             frc::Timer::GetFPGATimestamp() - timestamp);
        Logger::RecordOutput("AprilTagVision/Inst" +
                                 std::to_string(instanceIndex) + "/RobotPose",
                             robotPose);
        Logger::RecordOutput("AprilTagVision/Inst" +
                                 std::to_string(instanceIndex) + "/TagPoses",
                             tagPoses);
      }
    }

    std::map<int, RobotState::TxTyObservation> txTyObservations;
    for (size_t frameIndex = 0;
         frameIndex < aprilTagInputs[instanceIndex].timestamps.size();
         frameIndex++) {
      double timestamp = aprilTagInputs[instanceIndex].timestamps[frameIndex];
      std::vector<double> values =
          aprilTagInputs[instanceIndex].frames[frameIndex];
      int tagEstimationDataEndIndex = 0;
      switch (static_cast<int>(values[0])) {
      case 1:
        tagEstimationDataEndIndex = 8;
        break;
      case 2:
        tagEstimationDataEndIndex = 16;
        break;
      default:
        break;
      }

      for (size_t index = tagEstimationDataEndIndex + 1; index < values.size();
           index += 10) {
        std::vector<double> tx(4);
        std::vector<double> ty(4);
        for (int i = 0; i < 4; i++) {
          tx[i] = values[index + 1 + (2 * i)];
          ty[i] = values[index + 1 + (2 * i) + 1];
        }
        int tagId = static_cast<int>(values[index]);
        double distance = values[index + 9];

        txTyObservations[tagId] = RobotState::TxTyObservation(
            tagId, instanceIndex, tx, ty, distance, timestamp);
      }
    }

    for (const auto &observation : txTyObservations) {
      if (allTxTyObservations.find(observation.first) ==
              allTxTyObservations.end() ||
          observation.second.distance <
              allTxTyObservations[observation.first].distance) {
        allTxTyObservations[observation.first] = observation.second;
      }
    }

    for (size_t frameIndex = 0;
         frameIndex < objDetectInputs[instanceIndex].timestamps.size();
         frameIndex++) {
      std::vector<double> frame =
          objDetectInputs[instanceIndex].frames[frameIndex];
      for (size_t i = 0; i < frame.size(); i += 10) {
        if (frame[i + 1] > VisionConstants::objDetectConfidenceThreshold) {
          std::vector<double> tx(4);
          std::vector<double> ty(4);
          for (int z = 0; z < 4; z++) {
            tx[z] = frame[i + 2 + (2 * z)];
            ty[z] = frame[i + 2 + (2 * z) + 1];
          }
          allAlgaeTxTyObservations.emplace_back(
              instanceIndex, tx, ty,
              objDetectInputs[instanceIndex].timestamps[frameIndex]);
        }
      }
    }

    if (VisionConstants::enableInstanceLogging &&
        aprilTagInputs[instanceIndex].timestamps.empty()) {
      Logger::RecordOutput("AprilTagVision/Inst" +
                               std::to_string(instanceIndex) + "/RobotPose",
                           frc::Pose2d());
    }

    if (VisionConstants::enableInstanceLogging &&
        frc::Timer::GetFPGATimestamp() - lastFrameTimes[instanceIndex] >
            VisionConstants::targetLogTimeSecs) {
      Logger::RecordOutput("AprilTagVision/Inst" +
                               std::to_string(instanceIndex) + "/TagPoses",
                           std::vector<frc::Pose3d>());
    }
  }

  Logger::RecordOutput("AprilTagVision/RobotPoses", allRobotPoses);

  std::vector<frc::Pose3d> allTagPoses;
  for (const auto &detectionEntry : lastTagDetectionTimes) {
    if (frc::Timer::GetFPGATimestamp() - detectionEntry.second <
        VisionConstants::targetLogTimeSecs) {
      std::optional<frc::Pose3d> tagPose =
          aprilTagLayoutSupplier().GetLayout().GetTagPose(detectionEntry.first);
      if (tagPose.has_value()) {
        allTagPoses.push_back(tagPose.value());
      }
    }
  }
  Logger::RecordOutput("AprilTagVision/TagPoses", allTagPoses);

  std::sort(allVisionObservations.begin(), allVisionObservations.end(),
            [](const RobotState::VisionObservation &a,
               const RobotState::VisionObservation &b) {
              return a.timestamp < b.timestamp;
            });
  for (const auto &observation : allVisionObservations) {
    RobotState::GetInstance().AddVisionObservation(observation);
  }
  for (const auto &observation : allTxTyObservations) {
    RobotState::GetInstance().AddTxTyObservation(observation.second);
  }
  std::sort(allAlgaeTxTyObservations.begin(), allAlgaeTxTyObservations.end(),
            [](const RobotState::AlgaeTxTyObservation &a,
               const RobotState::AlgaeTxTyObservation &b) {
              return a.timestamp < b.timestamp;
            });
  for (const auto &observation : allAlgaeTxTyObservations) {
    RobotState::GetInstance().AddAlgaeTxTyObservation(observation);
  }

  LoggedTracer::Record("Vision");
}
