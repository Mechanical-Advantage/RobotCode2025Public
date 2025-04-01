// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "CoralSensorIOInputsAutoLogged.h"

class CoralSensorIO {
public:
  struct CoralSensorIOData {
    double distanceMeters = 0.0;
    bool valid = false;
  };

  virtual ~CoralSensorIO() = default;

  virtual void UpdateInputs(CoralSensorIOInputsAutoLogged &inputs) {}
};