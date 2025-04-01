// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "frc/Filesystem.h"
#include "frc/net/WebServer.h"
#include "networktables/BooleanPublisher.h"
#include "networktables/BooleanSubscriber.h"
#include "networktables/IntegerPublisher.h"
#include "networktables/IntegerSubscriber.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/PubSubOption.h"

#include "org/littletonrobotics/frc2025/subsystems/objectivetracker/ReefControlsIO.h"

#include <filesystem>

class ReefControlsIOServer : public ReefControlsIO {
public:
  ReefControlsIOServer();
  ~ReefControlsIOServer() override = default;

  void UpdateInputs(ReefControlsIOInputs &inputs) override;
  void SetSelectedLevel(int value) override;
  void SetLevel1State(int value) override;
  void SetLevel2State(int value) override;
  void SetLevel3State(int value) override;
  void SetLevel4State(int value) override;
  void SetAlgaeState(int value) override;
  void SetCoopState(bool value) override;
  void SetElims(bool isElims) override;

private:
  static constexpr char toRobotTable[] = "/ReefControls/ToRobot";
  static constexpr char toDashboardTable[] = "/ReefControls/ToDashboard";
  static constexpr char selectedLevelTopicName[] = "SelectedLevel";
  static constexpr char l1TopicName[] = "Level1";
  static constexpr char l2TopicName[] = "Level2";
  static constexpr char l3TopicName[] = "Level3";
  static constexpr char l4TopicName[] = "Level4";
  static constexpr char algaeTopicName[] = "Algae";
  static constexpr char coopTopicName[] = "Coop";
  static constexpr char isElimsTopicName[] = "IsElims";

  nt::IntegerSubscriber selectedLevelIn;
  nt::IntegerSubscriber l1StateIn;
  nt::IntegerSubscriber l2StateIn;
  nt::IntegerSubscriber l3StateIn;
  nt::IntegerSubscriber l4StateIn;
  nt::IntegerSubscriber algaeStateIn;
  nt::BooleanSubscriber coopStateIn;

  nt::IntegerPublisher selectedLevelOut;
  nt::IntegerPublisher l1StateOut;
  nt::IntegerPublisher l2StateOut;
  nt::IntegerPublisher l3StateOut;
  nt::IntegerPublisher l4StateOut;
  nt::IntegerPublisher algaeStateOut;
  nt::BooleanPublisher coopStateOut;
  nt::BooleanPublisher isElimsOut;
};