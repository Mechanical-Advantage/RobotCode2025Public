// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "frc2025/subsystems/rollers/RollerSystem.h"

#include <frc/Alert.h>
#include <frc2/command/Command.h>
#include <frc2/command/SubsystemBase.h>
#include <wpi/function.h>

#include "frc2025/Robot.h"
#include "frc2025/util/LoggedTracer.h"
#include "third_party/frc-wpi-util/wpi/StringRef.h"
#include "third_party/frc-wpi-util/wpi/function_ref.h"
#include "third_party/frc-wpi-util/wpi/span.h"
#include "third_party/frc-wpi-util/wpi/string.h"
#include "third_party/frc-wpi-util/wpi/units/voltage.h"

using namespace frc;
using namespace frc2;
using namespace wpi;

RollerSystem::RollerSystem(std::string name, RollerSystemIO *io)
    : SubsystemBase(), name_(name), io_(io),
      disconnected_(name + " motor disconnected!", Alert::AlertType::kWarning),
      tempFault_(name + " motor too hot! \xF0\x9F\x98\xB5",
                 Alert::AlertType::kWarning) {}

void RollerSystem::Periodic() {
  io_->UpdateInputs(&inputs_);
  Logger::ProcessInputs(name_, &inputs_);
  disconnected_.Set(!inputs_.data.connected && !Robot::IsJitting());
  tempFault_.Set(inputs_.data.tempFault);

  // Record cycle time
  LoggedTracer::Record(name_);
}

CommandPtr RollerSystem::RunRoller(std::function<double()> inputVolts) {
  return StartEnd([this, inputVolts] { io_->RunVolts(inputVolts()); },
                  [this] { io_->Stop(); });
}