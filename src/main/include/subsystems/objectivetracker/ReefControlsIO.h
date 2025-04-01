// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <vector>

#include "org/littletonrobotics/junction/AutoLog.h"

class ReefControlsIO {
public:
  struct ReefControlsIOInputs {
    std::vector<int> selectedLevel; // 0 = L2, 1 = L3, 2 = L4
    std::vector<int> level1State;   // Count
    std::vector<int> level2State;   // Bitfield
    std::vector<int> level3State;   // Bitfield
    std::vector<int> level4State;   // Bitfield
    std::vector<int> algaeState;    // Bitfield
    std::vector<bool> coopState;    // Boolean

    AUTO_LOG_STRUCT()
  };

  virtual ~ReefControlsIO() = default;

  virtual void UpdateInputs(ReefControlsIOInputs &inputs) {}

  virtual void SetSelectedLevel(int value) {}

  virtual void SetLevel1State(int value) {}

  virtual void SetLevel2State(int value) {}

  virtual void SetLevel3State(int value) {}

  virtual void SetLevel4State(int value) {}

  virtual void SetAlgaeState(int value) {}

  virtual void SetCoopState(bool value) {}

  virtual void SetElims(bool isElims) {}
};