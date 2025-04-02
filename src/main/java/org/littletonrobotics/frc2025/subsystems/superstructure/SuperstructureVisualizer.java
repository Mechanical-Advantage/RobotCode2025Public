// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.superstructure;

import static org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.Constants.Mode;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.util.EqualsUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperstructureVisualizer {
  private static final Translation2d dispenserOriginToAlgaeOrigin =
      new Translation2d(Units.inchesToMeters(14.157458), Rotation2d.fromDegrees(60.0));

  private final String name;
  private final LoggedMechanism2d mechanism =
      new LoggedMechanism2d(
          Units.inchesToMeters(28.0), Units.feetToMeters(7.0), new Color8Bit(Color.kDarkGray));
  private final LoggedMechanismLigament2d elevatorMechanism;
  private final LoggedMechanismLigament2d pivotMechanism;

  public SuperstructureVisualizer(String name) {
    this.name = name;
    LoggedMechanismRoot2d root =
        mechanism.getRoot(
            name + " Root", superstructureOrigin2d.getX(), superstructureOrigin2d.getY());
    elevatorMechanism =
        root.append(
            new LoggedMechanismLigament2d(
                name + " Elevator",
                Units.inchesToMeters(26.0),
                elevatorAngle.getDegrees(),
                4.0,
                new Color8Bit(Color.kFirstBlue)));
    pivotMechanism =
        elevatorMechanism.append(
            new LoggedMechanismLigament2d(
                name + " Pivot",
                Units.inchesToMeters(6.0),
                0.0,
                8.0,
                new Color8Bit(Color.kFirstRed)));
  }

  public void update(
      double elevatorHeightMeters, Rotation2d pivotFinalAngle, boolean hasAlgae, boolean hasCoral) {
    if (Constants.getMode() != Mode.REAL) {
      elevatorMechanism.setLength(
          EqualsUtil.epsilonEquals(elevatorHeightMeters, 0.0)
              ? Units.inchesToMeters(1.0)
              : elevatorHeightMeters);
      pivotMechanism.setAngle(pivotFinalAngle.minus(elevatorAngle));
      Logger.recordOutput("Mechanism2d/" + name, mechanism);
    }

    // Max of top of carriage or starting height
    final double heightFromBottom =
        elevatorHeightMeters + dispenserToBottom + dispenserToTop + stageThickness * 2.0;
    final double firstStageHeight =
        Math.max(
            heightFromBottom - SuperstructureConstants.firstStageHeight - stageThickness,
            stageThickness);
    final double secondStageHeight =
        Math.max(firstStageHeight - stageHeight + stageToStage - stageThickness, 0.0);

    Pose3d pivotPose3d =
        new Pose3d(
            dispenserOrigin3d.plus(
                new Translation3d(
                    elevatorHeightMeters, new Rotation3d(0.0, -elevatorAngle.getRadians(), 0.0))),
            new Rotation3d(
                0.0,
                // Have to invert angle due to CAD??
                -pivotFinalAngle.getRadians(),
                0.0));

    Logger.recordOutput(
        "Mechanism3d/" + name + "/Superstructure",
        new Pose3d(
            superstructureOrigin3d.plus(
                new Translation3d(
                    secondStageHeight, new Rotation3d(0.0, -elevatorAngle.getRadians(), 0.0))),
            Rotation3d.kZero),
        new Pose3d(
            superstructureOrigin3d.plus(
                new Translation3d(
                    firstStageHeight, new Rotation3d(0.0, -elevatorAngle.getRadians(), 0.0))),
            Rotation3d.kZero),
        new Pose3d(pivotPose3d.getTranslation(), Rotation3d.kZero),
        pivotPose3d);
    if (hasAlgae) {
      Logger.recordOutput(
          "Mechanism3d/" + name + "/Algae",
          new Translation3d[] {
            new Pose3d(RobotState.getInstance().getEstimatedPose())
                .transformBy(new Transform3d(Pose3d.kZero, pivotPose3d))
                .transformBy(
                    new Transform3d(
                        dispenserOriginToAlgaeOrigin.getX(),
                        0.0,
                        dispenserOriginToAlgaeOrigin.getY(),
                        Rotation3d.kZero))
                .getTranslation()
          });
    } else {
      Logger.recordOutput("Mechanism3d/" + name + "/Algae", new Translation3d[] {});
    }

    if (hasCoral) {
      Logger.recordOutput(
          "Mechanism3d/" + name + "/Coral",
          new Pose3d[] {
            new Pose3d(RobotState.getInstance().getEstimatedPose())
                .transformBy(new Transform3d(Pose3d.kZero, pivotPose3d))
                .transformBy(new Transform3d(pivotToTunnelFront, 0.0, 0, new Rotation3d(0, 0, 0)))
          });
    } else {
      Logger.recordOutput("Mechanism3d/" + name + "/Coral", new Pose3d[] {});
    }
  }
}
