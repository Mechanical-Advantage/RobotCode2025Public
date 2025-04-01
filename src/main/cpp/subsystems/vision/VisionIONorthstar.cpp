// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/vision/VisionIONorthstar.h"

#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <ntcore_cpp.h>
#include <wpi/Timestamp.h>

#include "FieldConstants.h"
#include "subsystems/vision/VisionConstants.h"
#include "util/SystemTimeValidReader.h"

VisionIONorthstar::VisionIONorthstar(
    std::function<FieldConstants::AprilTagLayoutType()> aprilTagLayoutSupplier,
    int index)
    : aprilTagLayoutSupplier(aprilTagLayoutSupplier),
      deviceId("northstar_" + std::to_string(index)),
      observationSubscriber(
          nt::NetworkTableInstance::GetDefault()
              .GetTable(deviceId)
              ->GetSubTable("output")
              ->GetDoubleArrayTopic("observations")
              .Subscribe(std::vector<double>{},
                         nt::PubSubOption::KeepDuplicates(true),
                         nt::PubSubOption::SendAll(true),
                         nt::PubSubOption::Periodic(0.01))),
      objDetectObservationSubscriber(
          nt::NetworkTableInstance::GetDefault()
              .GetTable(deviceId)
              ->GetSubTable("output")
              ->GetDoubleArrayTopic("objdetect_observations")
              .Subscribe(std::vector<double>{},
                         nt::PubSubOption::KeepDuplicates(true),
                         nt::PubSubOption::SendAll(true),
                         nt::PubSubOption::Periodic(0.01))),
      fpsAprilTagsSubscriber(nt::NetworkTableInstance::GetDefault()
                                 .GetTable(deviceId)
                                 ->GetSubTable("output")
                                 ->GetIntegerTopic("fps_apriltags")
                                 .Subscribe(0)),
      fpsObjDetectSubscriber(nt::NetworkTableInstance::GetDefault()
                                 .GetTable(deviceId)
                                 ->GetSubTable("output")
                                 ->GetIntegerTopic("fps_objdetect")
                                 .Subscribe(0)),
      timestampPublisher(nt::NetworkTableInstance::GetDefault()
                             .GetTable(deviceId)
                             ->GetSubTable("config")
                             ->GetIntegerTopic("timestamp")
                             .Publish()),
      isRecordingPublisher(nt::NetworkTableInstance::GetDefault()
                               .GetTable(deviceId)
                               ->GetSubTable("config")
                               ->GetBooleanTopic("is_recording")
                               .Publish()),
      tagLayoutPublisher(nt::NetworkTableInstance::GetDefault()
                             .GetTable(deviceId)
                             ->GetSubTable("config")
                             ->GetStringTopic("tag_layout")
                             .Publish()) {
  auto configTable =
      nt::NetworkTableInstance::GetDefault().GetTable(deviceId)->GetSubTable(
          "config");
  auto camera = VisionConstants::cameras[index];
  configTable->GetStringTopic("camera_id").Publish().Set(camera.id);
  configTable->GetIntegerTopic("camera_resolution_width")
      .Publish()
      .Set(camera.width);
  configTable->GetIntegerTopic("camera_resolution_height")
      .Publish()
      .Set(camera.height);
  configTable->GetIntegerTopic("camera_auto_exposure")
      .Publish()
      .Set(camera.autoExposure);
  configTable->GetIntegerTopic("camera_exposure")
      .Publish()
      .Set(camera.exposure);
  configTable->GetDoubleTopic("camera_gain").Publish().Set(camera.gain);
  configTable->GetDoubleTopic("fiducial_size_m")
      .Publish()
      .Set(FieldConstants::aprilTagWidth);

  isRecordingPublisher.Set(false);

  slowPeriodicTimer.Start();
}

void VisionIONorthstar::UpdateInputs(VisionIOInputs &inputs,
                                     AprilTagVisionIOInputs &aprilTagInputs,
                                     ObjDetectVisionIOInputs &objDetectInputs) {
  bool slowPeriodic = slowPeriodicTimer.AdvanceIfElapsed(1.0);

  inputs.ntConnected = false;
  for (const auto &client :
       nt::NetworkTableInstance::GetDefault().GetConnections()) {
    if (client.remote_id.starts_with(deviceId)) {
      inputs.ntConnected = true;
      break;
    }
  }

  if (slowPeriodic && SystemTimeValidReader::IsValid()) {
    timestampPublisher.Set(wpi::Now() / 1000000);
  }

  auto aprilTagType = aprilTagLayoutSupplier();
  if (aprilTagType != lastAprilTagLayout) {
    lastAprilTagLayout = aprilTagType;
    tagLayoutPublisher.Set(
        FieldConstants::GetAprilTagLayoutString(aprilTagType));
  }

  auto aprilTagQueue = observationSubscriber.ReadQueue();
  aprilTagInputs.timestamps.resize(aprilTagQueue.size());
  aprilTagInputs.frames.resize(aprilTagQueue.size());
  for (size_t i = 0; i < aprilTagQueue.size(); i++) {
    aprilTagInputs.timestamps[i] = aprilTagQueue[i].timestamp / 1000000.0;
    aprilTagInputs.frames[i] = aprilTagQueue[i].value;
  }
  if (slowPeriodic) {
    aprilTagInputs.fps = fpsAprilTagsSubscriber.Get();
  }

  auto objDetectQueue = objDetectObservationSubscriber.ReadQueue();
  objDetectInputs.timestamps.resize(objDetectQueue.size());
  objDetectInputs.frames.resize(objDetectQueue.size());
  for (size_t i = 0; i < objDetectQueue.size(); i++) {
    objDetectInputs.timestamps[i] = objDetectQueue[i].timestamp / 1000000.0;
    objDetectInputs.frames[i] = objDetectQueue[i].value;
  }
  if (slowPeriodic) {
    objDetectInputs.fps = fpsObjDetectSubscriber.Get();
  }
}

void VisionIONorthstar::SetRecording(bool active) {
  isRecordingPublisher.Set(active);
}