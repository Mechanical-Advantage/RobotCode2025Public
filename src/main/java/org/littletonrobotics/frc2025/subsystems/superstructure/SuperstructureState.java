// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructurePose.Preset;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureStateData.Height;
import org.littletonrobotics.frc2025.subsystems.superstructure.chariot.Chariot;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.Dispenser;

@Getter
@RequiredArgsConstructor
public enum SuperstructureState {
  START(SuperstructureStateData.builder().height(Height.BOTTOM).build()),
  CHARACTERIZATION(SuperstructureStateData.builder().build()),
  STOW(SuperstructureStateData.builder().pose(Preset.STOW.getPose()).height(Height.BOTTOM).build()),
  INTAKE(
      SuperstructureStateData.builder()
          .pose(Preset.INTAKE.getPose())
          .tunnelVolts(Dispenser.tunnelIntakeVolts)
          .build()),
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
  L1_CORAL_EJECT(
      L1_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts).build()),
  L2_CORAL_EJECT(
      L2_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts).build()),
  L3_CORAL_EJECT(
      L3_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts).build()),
  L4_CORAL_EJECT(
      L4_CORAL.getValue().toBuilder().tunnelVolts(Dispenser.tunnelDispenseVolts).build()),
  ALGAE_FLOOR_INTAKE(
      SuperstructureStateData.builder()
          .pose(Preset.ALGAE_FLOOR_INTAKE.getPose())
          .chariotGoal(Chariot.Goal.DEPLOY)
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .intakeVolts(Chariot.floorIntakeVolts)
          .build()),
  ALGAE_L2_INTAKE(
      SuperstructureStateData.builder()
          .pose(Preset.ALGAE_L2_INTAKE.getPose())
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.FIRST_STAGE)
          .build()),
  ALGAE_L3_INTAKE(
      SuperstructureStateData.builder()
          .pose(Preset.ALGAE_L3_INTAKE.getPose())
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.FIRST_STAGE)
          .build()),
  PRE_THROWN(
      SuperstructureStateData.builder()
          .pose(Preset.PRE_THROW.getPose())
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.SECOND_STAGE)
          .build()),
  THROWN(
      SuperstructureStateData.builder()
          .pose(Preset.THROW.getPose())
          .gripperCurrent(Dispenser.gripperDispenseCurrent)
          .build()),
  ALGAE_STOW(
      SuperstructureStateData.builder()
          .pose(Preset.ALGAE_STOW.getPose())
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .build()),
  PRE_TOSS(
      SuperstructureStateData.builder()
          .pose(
              new SuperstructurePose(
                  Preset.ALGAE_STOW.getPose().elevatorHeight(), () -> Rotation2d.fromDegrees(30.0)))
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .build()),
  ALGAE_STOW_INTAKE(SuperstructureState.ALGAE_STOW.getValue().toBuilder().build()),
  TOSS(
      SuperstructureState.PRE_TOSS.getValue().toBuilder()
          .gripperCurrent(Dispenser.gripperDispenseCurrent)
          .build()),
  POST_PRE_PROCESSOR(
      SuperstructureStateData.builder()
          .pose(Preset.POST_PRE_PROCESSOR.getPose())
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .build()),
  PROCESSED(
      POST_PRE_PROCESSOR.getValue().toBuilder()
          .gripperCurrent(Dispenser.gripperDispenseCurrent)
          .intakeVolts(Chariot.occupiedVolts)
          .build()),
  L1_CORAL_REVERSED(
      SuperstructureStateData.builder()
          .pose(Preset.L1_CORAL_REVERSED.getPose())
          .reversed(true)
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.FIRST_STAGE)
          .build()),
  L2_CORAL_REVERSED(
      SuperstructureStateData.builder()
          .pose(Preset.L2_CORAL_REVERSED.getPose())
          .reversed(true)
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.FIRST_STAGE)
          .build()),
  L3_CORAL_REVERSED(
      SuperstructureStateData.builder()
          .pose(Preset.L3_CORAL_REVERSED.getPose())
          .reversed(true)
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.FIRST_STAGE)
          .build()),
  L4_CORAL_REVERSED(
      SuperstructureStateData.builder()
          .pose(Preset.L4_CORAL_REVERSED.getPose())
          .reversed(true)
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.SECOND_STAGE)
          .build()),
  L1_CORAL_REVERSED_EJECT(
      L1_CORAL_REVERSED.getValue().toBuilder().tunnelVolts(Dispenser.tunnelIntakeVolts).build()),
  L2_CORAL_REVERSED_EJECT(
      L2_CORAL_REVERSED.getValue().toBuilder().tunnelVolts(Dispenser.tunnelIntakeVolts).build()),
  L3_CORAL_REVERSED_EJECT(
      L3_CORAL_REVERSED.getValue().toBuilder().tunnelVolts(Dispenser.tunnelIntakeVolts).build()),
  L4_CORAL_REVERSED_EJECT(
      L4_CORAL_REVERSED.getValue().toBuilder().tunnelVolts(Dispenser.tunnelIntakeVolts).build()),
  UNREVERSED(
      SuperstructureStateData.builder()
          .pose(new SuperstructurePose(() -> 0.0, () -> SuperstructureConstants.pivotSafeAngle))
          .gripperCurrent(Dispenser.gripperIntakeCurrent)
          .height(Height.SECOND_STAGE) // Always above second stage
          .build());

  private final SuperstructureStateData value;
}
