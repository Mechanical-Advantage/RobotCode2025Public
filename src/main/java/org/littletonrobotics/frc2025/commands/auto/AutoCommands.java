// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.commands.AutoScoreCommands;
import org.littletonrobotics.frc2025.commands.DriveTrajectory;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.MirrorUtil;

public class AutoCommands {
  private AutoCommands() {}

  public static Command resetPoseCommand(HolonomicTrajectory trajectory, boolean mirror) {
    return Commands.runOnce(
        () ->
            RobotState.getInstance()
                .resetPose(
                    AllianceFlipUtil.apply(
                        mirror
                            ? MirrorUtil.apply(trajectory.getStartPose())
                            : trajectory.getStartPose())));
  }

  public static void resetPose(HolonomicTrajectory trajectory, boolean mirror) {
    RobotState.getInstance()
        .resetPose(
            AllianceFlipUtil.apply(
                mirror ? MirrorUtil.apply(trajectory.getStartPose()) : trajectory.getStartPose()));
  }

  public static DriveTrajectory coralScoringTrajectory(
      Drive drive, HolonomicTrajectory trajectory, CoralObjective coralObjective, boolean mirror) {
    return new DriveTrajectory(
        drive,
        trajectory,
        () ->
            AutoScoreCommands.getRobotPose(
                mirror ? MirrorUtil.apply(coralObjective) : coralObjective, false),
        mirror);
  }

  public static Command driveAimAtBranch(
      DriveTrajectory trajectoryCommand, Supplier<CoralObjective> coralObjective) {
    return Commands.run(
            () ->
                trajectoryCommand.setOverrideRotation(
                    Optional.of(
                        AllianceFlipUtil.apply(
                                AutoScoreCommands.getBranchPose(coralObjective.get()))
                            .getTranslation()
                            .minus(
                                AutoScoreCommands.getRobotPose(coralObjective.get(), false)
                                    .getTranslation())
                            .getAngle())))
        .finallyDo(() -> trajectoryCommand.setOverrideRotation(Optional.empty()));
  }

  public static Command superstructureAimAndEjectCommand(
      Superstructure superstructure,
      CoralObjective coralObjective,
      boolean mirror,
      BooleanSupplier eject) {
    return AutoScoreCommands.superstructureAimAndEject(
        superstructure,
        coralObjective::reefLevel,
        () -> Optional.of(mirror ? MirrorUtil.apply(coralObjective) : coralObjective),
        eject);
  }

  public static boolean isXCrossed(double x, boolean towardsDriverStation) {
    double flippedX = AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose()).getX();
    return towardsDriverStation ? flippedX <= x : flippedX >= x;
  }
}
