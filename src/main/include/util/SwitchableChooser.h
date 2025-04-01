// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "thirdparty/junction/LoggedNetworkInput.h"
#include "thirdparty/junction/networktables/LoggedNetworkString.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringArrayPublisher.h>
#include <networktables/StringPublisher.h>
#include <string>
#include <vector>

namespace frc2025::util {

class SwitchableChooser : public org::littletonrobotics::junction::
                              networktables::LoggedNetworkInput {
public:
  SwitchableChooser(std::string name);

  /** Updates the set of available options. */
  void setOptions(std::vector<std::string> options);

  /** Returns the selected option. */
  std::string get();

  void periodic() override;

private:
  static const std::string placeholder;

  std::vector<std::string> options;
  std::string active;

  nt::StringPublisher namePublisher;
  nt::StringPublisher typePublisher;
  nt::StringArrayPublisher optionsPublisher;
  nt::StringPublisher defaultPublisher;
  nt::StringPublisher activePublisher;
  nt::StringPublisher selectedPublisher;
  org::littletonrobotics::junction::networktables::LoggedNetworkString
      selectedInput;
};

} // namespace frc2025::util