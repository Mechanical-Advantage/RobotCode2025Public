// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.*;
import java.util.stream.Collectors;
import lombok.Getter;
import org.littletonrobotics.frc2025.FieldConstants;

public class AutoCoralSim {
  private static final double intakeWidth = Units.inchesToMeters(28.0);
  private static final LoggedTunableNumber intakeDistance =
      new LoggedTunableNumber("CoralSim/IntakeDistance", 0.15);
  @Getter private static final Set<Translation2d> corals = new HashSet<>();

  public static void resetCorals() {
    corals.clear();
    corals.addAll(
        Arrays.stream(FieldConstants.StagingPositions.iceCreams)
            .map(AllianceFlipUtil::apply)
            .collect(Collectors.toSet()));
    // Add coral to station
    Random random = new Random();
    for (int i = 0; i < 3; i++) {
      corals.add(
          AllianceFlipUtil.apply(
              MirrorUtil.apply(
                  FieldConstants.CoralStation.rightCenterFace
                      .transformBy(
                          GeomUtil.toTransform2d(
                              0.5 + random.nextDouble() * 2.0, 0.5 - random.nextDouble() * 2.0))
                      .getTranslation())));
    }
  }

  public static boolean intakeCoral(Pose2d intakePose) {
    var intakedCoral =
        corals.stream()
            .filter(
                coral -> {
                  var offset = new Pose2d(coral, intakePose.getRotation()).relativeTo(intakePose);
                  return offset.getX() <= intakeDistance.get()
                      && offset.getX() >= -0.05
                      && Math.abs(offset.getY()) <= intakeWidth / 2.0;
                })
            .min(
                Comparator.comparingDouble(
                    coral -> intakePose.getTranslation().getDistance(coral)));
    if (intakedCoral.isPresent()) {
      corals.remove(intakedCoral.get());
      return true;
    } else {
      return false;
    }
  }
}
