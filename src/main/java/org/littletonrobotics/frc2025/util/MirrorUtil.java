// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.BooleanSupplier;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.CoralObjective;
import org.littletonrobotics.frc2025.subsystems.drive.trajectory.TrajectoryGenerationHelpers;
import org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

@ExtensionMethod({TrajectoryGenerationHelpers.class})
public class MirrorUtil {
  @Setter private static BooleanSupplier mirror;

  public static CoralObjective apply(CoralObjective coralObjective) {
    if (!shouldMirror()) return coralObjective;
    int shiftedBranchId = coralObjective.branchId() - 1;
    if (shiftedBranchId == -1) {
      shiftedBranchId = 11;
    }
    int flippedBranchId = 11 - shiftedBranchId;
    flippedBranchId = (++flippedBranchId == 12) ? 0 : flippedBranchId;
    return new CoralObjective(flippedBranchId, coralObjective.reefLevel());
  }

  public static double apply(double y) {
    return shouldMirror() ? FieldConstants.fieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    if (!shouldMirror()) return translation;
    return new Translation2d(translation.getX(), apply(translation.getY()));
  }

  public static Pose2d apply(Pose2d pose) {
    if (!shouldMirror()) return pose;
    return new Pose2d(
        apply(pose.getTranslation()),
        new Rotation2d(pose.getRotation().getCos(), -pose.getRotation().getSin()));
  }

  public static VehicleState apply(VehicleState state) {
    if (!shouldMirror()) return state;
    Pose2d pose = apply(state.getPose());
    return VehicleState.newBuilder()
        .setX(pose.getX())
        .setY(pose.getY())
        .setTheta(pose.getRotation().getRadians())
        .setVx(state.getVx())
        .setVy(-state.getVy())
        .setOmega(-state.getOmega())
        .addAllModuleForces(
            state.getModuleForcesList().stream()
                .map(
                    forces ->
                        ModuleForce.newBuilder()
                            .setFx(forces.getFx())
                            .setFy(-forces.getFy())
                            .build())
                .toList())
        .build();
  }

  public static boolean shouldMirror() {
    return mirror != null && mirror.getAsBoolean();
  }
}
