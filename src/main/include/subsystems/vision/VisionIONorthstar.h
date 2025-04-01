// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <string>
#include <vector>

#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <ntcore_cpp.h>
#include <wpi/Timestamp.h>

#include "FieldConstants.h"
#include "subsystems/vision/VisionConstants.h"
#include "subsystems/vision/VisionIO.h"
#include "util/SystemTimeValidReader.h"

class VisionIONorthstar : public VisionIO {
public:
  VisionIONorthstar(std::function<FieldConstants::AprilTagLayoutType()>
                        aprilTagLayoutSupplier,
                    int index);

  void UpdateInputs(VisionIOInputs &inputs,
                    AprilTagVisionIOInputs &aprilTagInputs,
                    ObjDetectVisionIOInputs &objDetectInputs) override;
  void SetRecording(bool active) override;

private:
  std::function<FieldConstants::AprilTagLayoutType()> aprilTagLayoutSupplier;
  FieldConstants::AprilTagLayoutType lastAprilTagLayout =
      FieldConstants::AprilTagLayoutType::k2024;

  std::string deviceId;
  nt::DoubleArraySubscriber observationSubscriber;
  nt::DoubleArraySubscriber objDetectObservationSubscriber;
  nt::IntegerSubscriber fpsAprilTagsSubscriber;
  nt::IntegerSubscriber fpsObjDetectSubscriber;
  nt::IntegerPublisher timestampPublisher;
  nt::BooleanPublisher isRecordingPublisher;
  nt::StringPublisher tagLayoutPublisher;

  frc::Timer slowPeriodicTimer;
};