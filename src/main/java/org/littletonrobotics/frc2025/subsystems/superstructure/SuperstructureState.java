// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructurePose.Preset;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureStateData.Height;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.Dispenser;

@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
  START(SuperstructureStateData.builder().build()),
  AUTO_START(SuperstructureStateData.builder().build()),
  CHARACTERIZATION(SuperstructureStateData.builder().build()),
  SAFETY(SuperstructureStateData.builder().build()),
  STOW(SuperstructureStateData.builder().pose(Preset.STOW.getPose()).build()),
  CORAL_INTAKE(
      SuperstructureStateData.builder()
          .pose(Preset.CORAL_INTAKE.getPose())
          .tunnelVolts(Dispenser.tunnelIntakeVolts)
          .build()),
  GOODBYE_CORAL(SuperstructureStateData.builder().pose(Preset.GOODBYE_CORAL.getPose()).build()),
  L1_CORAL(SuperstructureStateData.builder().pose(Preset.L1.getPose()).build()),
  L2_CORAL(
      SuperstructureStateData.builder()
          .pose(Preset.L2.getPose())
          .height(Height.FIRST_STAGE)
          .build()),
  L3_CORAL(
      SuperstructureStateData.builder()
          .pose(Preset.L3.getPose())
          .height(Height.FIRST_STAGE)
          .build()),
  L4_CORAL(
      SuperstructureStateData.builder()
          .pose(Preset.L4.getPose())
          .height(Height.SECOND_STAGE)
          .build()),
  GOODBYE_CORAL_EJECT(
      GOODBYE_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts[3]).build()),
  L1_CORAL_EJECT(
      SuperstructureStateData.builder()
          .pose(Preset.L1_EJECT.getPose())
          .tunnelVolts(Dispenser.tunnelDispenseVolts[0])
          .build()),
  L2_CORAL_EJECT(
      L2_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts[1]).build()),
  L3_CORAL_EJECT(
      L3_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts[2]).build()),
  L4_CORAL_EJECT(
      L4_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts[3]).build()),
  ALGAE_STOW(
      SuperstructureStateData.builder()
          .pose(Preset.ALGAE_STOW.getPose())
          .gripperGoal(Dispenser.GripperGoal.GRIP)
          .build()),
  ALGAE_CORAL_INTAKE(
      CORAL_INTAKE.getValue().toBuilder().gripperGoal(Dispenser.GripperGoal.GRIP).build()),
  ALGAE_GOODBYE_CORAL(
      GOODBYE_CORAL.getValue().toBuilder().gripperGoal(Dispenser.GripperGoal.GRIP).build()),
  ALGAE_L2_CORAL(
      SuperstructureStateData.builder()
          .pose(Preset.ALGAE_L2.getPose())
          .gripperGoal(Dispenser.GripperGoal.GRIP)
          .height(Height.FIRST_STAGE)
          .build()),
  ALGAE_L3_CORAL(
      SuperstructureStateData.builder()
          .pose(Preset.ALGAE_L3.getPose())
          .gripperGoal(Dispenser.GripperGoal.GRIP)
          .height(Height.FIRST_STAGE)
          .build()),
  ALGAE_L4_CORAL(
      SuperstructureStateData.builder()
          .pose(Preset.ALGAE_L4.getPose())
          .gripperGoal(Dispenser.GripperGoal.GRIP)
          .height(Height.SECOND_STAGE)
          .build()),
  ALGAE_GOODBYE_CORAL_EJECT(
      ALGAE_GOODBYE_CORAL.getValue().toBuilder()
          .tunnelVolts(Dispenser.tunnelDispenseVolts[3])
          .build()),
  ALGAE_L2_CORAL_EJECT(
      ALGAE_L2_CORAL.getValue().toBuilder()
          .tunnelVolts(Dispenser.tunnelDispenseVoltsAlgae[0])
          .build()),
  ALGAE_L3_CORAL_EJECT(
      ALGAE_L3_CORAL.getValue().toBuilder()
          .tunnelVolts(Dispenser.tunnelDispenseVoltsAlgae[1])
          .build()),
  ALGAE_L4_CORAL_EJECT(
      ALGAE_L4_CORAL.getValue().toBuilder()
          .tunnelVolts(Dispenser.tunnelDispenseVoltsAlgae[2])
          .build()),
  ALGAE_L2_INTAKE(
      SuperstructureStateData.builder()
          .pose(Preset.ALGAE_L2_INTAKE.getPose())
          .gripperGoal(Dispenser.GripperGoal.GRIP)
          .height(Height.FIRST_STAGE)
          .build()),
  ALGAE_L3_INTAKE(
      SuperstructureStateData.builder()
          .pose(Preset.ALGAE_L3_INTAKE.getPose())
          .gripperGoal(Dispenser.GripperGoal.GRIP)
          .height(Height.FIRST_STAGE)
          .build()),
  ALGAE_ICE_CREAM_INTAKE(
      SuperstructureStateData.builder()
          .pose(Preset.ALGAE_ICE_CREAM_INTAKE.getPose())
          .gripperGoal(Dispenser.GripperGoal.GRIP)
          .build()),
  PRE_THROW(
      SuperstructureStateData.builder()
          .pose(Preset.THROW.getPose())
          .gripperGoal(Dispenser.GripperGoal.GRIP)
          .height(Height.SECOND_STAGE)
          .build()),
  THROW(PRE_THROW.getValue().toBuilder().gripperGoal(Dispenser.GripperGoal.EJECT).build()),
  TOSS(
      SuperstructureState.ALGAE_STOW.getValue().toBuilder()
          .gripperGoal(Dispenser.GripperGoal.EJECT)
          .build()),
  ALGAE_STOW_INTAKE(SuperstructureState.ALGAE_STOW.getValue().toBuilder().build()),
  PRE_PROCESS(
      SuperstructureStateData.builder()
          .pose(Preset.PROCESS.getPose())
          .gripperGoal(Dispenser.GripperGoal.GRIP)
          .build()),
  PROCESS(
      SuperstructureState.PRE_PROCESS.getValue().toBuilder()
          .gripperGoal(Dispenser.GripperGoal.EJECT)
          .build());

  private final SuperstructureStateData value;
}
