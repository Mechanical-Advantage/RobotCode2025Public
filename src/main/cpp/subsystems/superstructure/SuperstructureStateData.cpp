// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/superstructure/SuperstructureStateData.h"

SuperstructureStateData::Builder::Builder()
    : pose(SuperstructurePose()),
      tunnelVolts(std::function<double()>([]() { return 0.0; })),
      gripperGoal(Dispenser::GripperGoal::IDLE),
      intakeVolts(std::function<double()>([]() { return 0.0; })),
      chariotGoal(Chariot::Goal::RETRACT),
      height(SuperstructureStateData::Height::BOTTOM), reversed(false) {}

SuperstructureStateData::Builder &
SuperstructureStateData::Builder::Pose(const SuperstructurePose &pose) {
  this->pose = pose;
  return *this;
}

SuperstructureStateData::Builder &SuperstructureStateData::Builder::TunnelVolts(
    std::function<double()> tunnelVolts) {
  this->tunnelVolts = tunnelVolts;
  return *this;
}

SuperstructureStateData::Builder &SuperstructureStateData::Builder::GripperGoal(
    Dispenser::GripperGoal gripperGoal) {
  this->gripperGoal = gripperGoal;
  return *this;
}

SuperstructureStateData::Builder &SuperstructureStateData::Builder::IntakeVolts(
    std::function<double()> intakeVolts) {
  this->intakeVolts = intakeVolts;
  return *this;
}

SuperstructureStateData::Builder &
SuperstructureStateData::Builder::ChariotGoal(Chariot::Goal chariotGoal) {
  this->chariotGoal = chariotGoal;
  return *this;
}

SuperstructureStateData::Builder &
SuperstructureStateData::Builder::Height(Height height) {
  this->height = height;
  return *this;
}

SuperstructureStateData::Builder &
SuperstructureStateData::Builder::Reversed(bool reversed) {
  this->reversed = reversed;
  return *this;
}

SuperstructureStateData SuperstructureStateData::Builder::Build() {
  SuperstructureStateData data;
  data.pose = pose;
  data.tunnelVolts = tunnelVolts;
  data.gripperGoal = gripperGoal;
  data.intakeVolts = intakeVolts;
  data.chariotGoal = chariotGoal;
  data.height = height;
  data.reversed = reversed;
  return data;
}

SuperstructureStateData::Builder SuperstructureStateData::BuilderCreate() {
  return Builder();
}

SuperstructureStateData::Builder SuperstructureStateData::ToBuilder() const {
  Builder builder;
  builder.pose = pose;
  builder.tunnelVolts = tunnelVolts;
  builder.gripperGoal = gripperGoal;
  builder.intakeVolts = intakeVolts;
  builder.chariotGoal = chariotGoal;
  builder.height = height;
  builder.reversed = reversed;
  return builder;
}