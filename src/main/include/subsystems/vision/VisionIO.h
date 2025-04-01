// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <vector>

#include "thirdparty/junction/AutoLog.h"

class VisionIO {
public:
  struct VisionIOInputs {
    JUnction_AUTO_LOG_VARS() bool ntConnected = false;
  };

  struct AprilTagVisionIOInputs {
    JUnction_AUTO_LOG_VARS() std::vector<double> timestamps;
    std::vector<std::vector<double>> frames;
    long fps = 0;
  };

  struct ObjDetectVisionIOInputs {
    JUnction_AUTO_LOG_VARS() std::vector<double> timestamps;
    std::vector<std::vector<double>> frames;
    long fps = 0;
  };

  virtual void UpdateInputs(VisionIOInputs &inputs,
                            AprilTagVisionIOInputs &aprilTagInputs,
                            ObjDetectVisionIOInputs &objDetectInputs) {}
  virtual void SetRecording(bool active) {}

  virtual ~VisionIO() = default;
};