// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "org/littletonrobotics/frc2025/subsystems/objectivetracker/ReefControlsIOServer.h"

#include "frc/Filesystem.h"
#include "frc/net/WebServer.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/PubSubOption.h"

#include <filesystem>

ReefControlsIOServer::ReefControlsIOServer()
    : selectedLevelIn(
          nt::NetworkTableInstance::GetDefault()
              .GetTable(toRobotTable)
              ->GetIntegerTopic(selectedLevelTopicName)
              .Subscribe(0, nt::PubSubOption::KeepDuplicates(true))),
      l1StateIn(nt::NetworkTableInstance::GetDefault()
                    .GetTable(toRobotTable)
                    ->GetIntegerTopic(l1TopicName)
                    .Subscribe(0, nt::PubSubOption::KeepDuplicates(true))),
      l2StateIn(nt::NetworkTableInstance::GetDefault()
                    .GetTable(toRobotTable)
                    ->GetIntegerTopic(l2TopicName)
                    .Subscribe(0, nt::PubSubOption::KeepDuplicates(true))),
      l3StateIn(nt::NetworkTableInstance::GetDefault()
                    .GetTable(toRobotTable)
                    ->GetIntegerTopic(l3TopicName)
                    .Subscribe(0, nt::PubSubOption::KeepDuplicates(true))),
      l4StateIn(nt::NetworkTableInstance::GetDefault()
                    .GetTable(toRobotTable)
                    ->GetIntegerTopic(l4TopicName)
                    .Subscribe(0, nt::PubSubOption::KeepDuplicates(true))),
      algaeStateIn(nt::NetworkTableInstance::GetDefault()
                       .GetTable(toRobotTable)
                       ->GetIntegerTopic(algaeTopicName)
                       .Subscribe(0, nt::PubSubOption::KeepDuplicates(true))),
      coopStateIn(
          nt::NetworkTableInstance::GetDefault()
              .GetTable(toRobotTable)
              ->GetBooleanTopic(coopTopicName)
              .Subscribe(false, nt::PubSubOption::KeepDuplicates(true))),
      selectedLevelOut(nt::NetworkTableInstance::GetDefault()
                           .GetTable(toDashboardTable)
                           ->GetIntegerTopic(selectedLevelTopicName)
                           .Publish()),
      l1StateOut(nt::NetworkTableInstance::GetDefault()
                     .GetTable(toDashboardTable)
                     ->GetIntegerTopic(l1TopicName)
                     .Publish()),
      l2StateOut(nt::NetworkTableInstance::GetDefault()
                     .GetTable(toDashboardTable)
                     ->GetIntegerTopic(l2TopicName)
                     .Publish()),
      l3StateOut(nt::NetworkTableInstance::GetDefault()
                     .GetTable(toDashboardTable)
                     ->GetIntegerTopic(l3TopicName)
                     .Publish()),
      l4StateOut(nt::NetworkTableInstance::GetDefault()
                     .GetTable(toDashboardTable)
                     ->GetIntegerTopic(l4TopicName)
                     .Publish()),
      algaeStateOut(nt::NetworkTableInstance::GetDefault()
                        .GetTable(toDashboardTable)
                        ->GetIntegerTopic(algaeTopicName)
                        .Publish()),
      coopStateOut(nt::NetworkTableInstance::GetDefault()
                       .GetTable(toDashboardTable)
                       ->GetBooleanTopic(coopTopicName)
                       .Publish()),
      isElimsOut(nt::NetworkTableInstance::GetDefault()
                     .GetTable(toDashboardTable)
                     ->GetBooleanTopic(isElimsTopicName)
                     .Publish()) {
  // Start web server
  frc::WebServer::Start(
      5801,
      std::filesystem::path(
          frc::Filesystem::GetDeployDirectory().GetAbsolutePath().string()) /
          "reefcontrols");
}

void ReefControlsIOServer::UpdateInputs(ReefControlsIOInputs &inputs) {
  if (selectedLevelIn.ReadQueue().size() > 0) {
    inputs.selectedLevel = {static_cast<int>(selectedLevelIn.Get())};
  }
  if (l1StateIn.ReadQueue().size() > 0) {
    inputs.level1State = {static_cast<int>(l1StateIn.Get())};
  }
  if (l2StateIn.ReadQueue().size() > 0) {
    inputs.level2State = {static_cast<int>(l2StateIn.Get())};
  }
  if (l3StateIn.ReadQueue().size() > 0) {
    inputs.level3State = {static_cast<int>(l3StateIn.Get())};
  }
  if (l4StateIn.ReadQueue().size() > 0) {
    inputs.level4State = {static_cast<int>(l4StateIn.Get())};
  }
  if (algaeStateIn.ReadQueue().size() > 0) {
    inputs.algaeState = {static_cast<int>(algaeStateIn.Get())};
  }
  if (coopStateIn.ReadQueue().size() > 0) {
    inputs.coopState = {coopStateIn.Get()};
  }
}

void ReefControlsIOServer::SetSelectedLevel(int value) {
  selectedLevelOut.Set(value);
}

void ReefControlsIOServer::SetLevel1State(int value) { l1StateOut.Set(value); }

void ReefControlsIOServer::SetLevel2State(int value) { l2StateOut.Set(value); }

void ReefControlsIOServer::SetLevel3State(int value) { l3StateOut.Set(value); }

void ReefControlsIOServer::SetLevel4State(int value) { l4StateOut.Set(value); }

void ReefControlsIOServer::SetAlgaeState(int value) {
  algaeStateOut.Set(value);
}

void ReefControlsIOServer::SetCoopState(bool value) { coopStateOut.Set(value); }

void ReefControlsIOServer::SetElims(bool isElims) { isElimsOut.Set(isElims); }