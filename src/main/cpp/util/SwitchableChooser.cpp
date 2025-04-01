// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "SwitchableChooser.h"
#include "thirdparty/junction/Logger.h"
#include <algorithm>

namespace frc2025::util {

const std::string SwitchableChooser::placeholder = "<NA>";

SwitchableChooser::SwitchableChooser(std::string name)
    : options({placeholder}), active(placeholder),
      namePublisher(nt::NetworkTableInstance::GetDefault()
                        .GetTable("SmartDashboard")
                        ->GetStringTopic(".name")
                        .Publish()),
      typePublisher(nt::NetworkTableInstance::GetDefault()
                        .GetTable("SmartDashboard")
                        ->GetStringTopic(".type")
                        .Publish()),
      optionsPublisher(nt::NetworkTableInstance::GetDefault()
                           .GetTable("SmartDashboard")
                           ->GetStringArrayTopic("options")
                           .Publish()),
      defaultPublisher(nt::NetworkTableInstance::GetDefault()
                           .GetTable("SmartDashboard")
                           ->GetStringTopic("default")
                           .Publish()),
      activePublisher(nt::NetworkTableInstance::GetDefault()
                          .GetTable("SmartDashboard")
                          ->GetStringTopic("active")
                          .Publish()),
      selectedPublisher(nt::NetworkTableInstance::GetDefault()
                            .GetTable("SmartDashboard")
                            ->GetStringTopic("selected")
                            .Publish()),
      selectedInput("/SmartDashboard/" + name + "/selected") {
  org::littletonrobotics::junction::Logger::RegisterDashboardInput(this);

  namePublisher.Set(name);
  typePublisher.Set("String Chooser");
  optionsPublisher.Set(options);
  defaultPublisher.Set(options[0]);
  activePublisher.Set(options[0]);
  selectedPublisher.Set(options[0]);
}

void SwitchableChooser::setOptions(std::vector<std::string> options) {
  if (this->options == options) {
    return;
  }
  this->options =
      options.empty() ? std::vector<std::string>({placeholder}) : options;
  optionsPublisher.Set(this->options);
  periodic();
}

std::string SwitchableChooser::get() {
  return active == placeholder ? "" : active;
}

void SwitchableChooser::periodic() {
  std::string selected = selectedInput.Get();
  active = "";
  for (const std::string &option : options) {
    if (option != placeholder && option == selected) {
      active = option;
      break;
    }
  }
  if (active.empty()) {
    active = options[0];
    selectedPublisher.Set(active);
  }
  defaultPublisher.Set(active);
  activePublisher.Set(active);
}

} // namespace frc2025::util