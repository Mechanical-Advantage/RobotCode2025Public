// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Twist2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

namespace util {

/** Geometry utilities for working with translations, rotations, transforms, and
 * poses. */
class GeomUtil {
public:
  /**
   * Creates a pure translating transform
   *
   * @param translation The translation to create the transform with
   * @return The resulting transform
   */
  static frc::geometry::Transform2d
  ToTransform2d(frc::geometry::Translation2d translation);

  /**
   * Creates a pure translating transform
   *
   * @param x The x coordinate of the translation
   * @param y The y coordinate of the translation
   * @return The resulting transform
   */
  static frc::geometry::Transform2d ToTransform2d(double x, double y);

  /**
   * Creates a pure rotating transform
   *
   * @param rotation The rotation to create the transform with
   * @return The resulting transform
   */
  static frc::geometry::Transform2d
  ToTransform2d(frc::geometry::Rotation2d rotation);

  /**
   * Converts a Pose2d to a Transform2d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  static frc::geometry::Transform2d ToTransform2d(frc::geometry::Pose2d pose);

  static frc::geometry::Pose2d Inverse(frc::geometry::Pose2d pose);

  /**
   * Converts a Transform2d to a Pose2d to be used as a position or as the start
   * of a kinematic chain
   *
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  static frc::geometry::Pose2d ToPose2d(frc::geometry::Transform2d transform);

  /**
   * Creates a pure translated pose
   *
   * @param translation The translation to create the pose with
   * @return The resulting pose
   */
  static frc::geometry::Pose2d
  ToPose2d(frc::geometry::Translation2d translation);

  /**
   * Creates a pure rotated pose
   *
   * @param rotation The rotation to create the pose with
   * @return The resulting pose
   */
  static frc::geometry::Pose2d ToPose2d(frc::geometry::Rotation2d rotation);

  /**
   * Multiplies a twist by a scaling factor
   *
   * @param twist The twist to multiply
   * @param factor The scaling factor for the twist components
   * @return The new twist
   */
  static frc::geometry::Twist2d Multiply(frc::geometry::Twist2d twist,
                                         double factor);

  /**
   * Converts a Pose3d to a Transform3d to be used in a kinematic chain
   *
   * @param pose The pose that will represent the transform
   * @return The resulting transform
   */
  static frc::geometry::Transform3d ToTransform3d(frc::geometry::Pose3d pose);

  /**
   * Converts a Transform3d to a Pose3d to be used as a position or as the start
   * of a kinematic chain
   *
   * @param transform The transform that will represent the pose
   * @return The resulting pose
   */
  static frc::geometry::Pose3d ToPose3d(frc::geometry::Transform3d transform);

  /**
   * Converts a ChassisSpeeds to a Twist2d by extracting two dimensions (Y and
   * Z). chain
   *
   * @param speeds The original translation
   * @return The resulting translation
   */
  static frc::geometry::Twist2d
  ToTwist2d(frc::kinematics::ChassisSpeeds speeds);

  /**
   * Creates a new pose from an existing one using a different translation
   * value.
   *
   * @param pose The original pose
   * @param translation The new translation to use
   * @return The new pose with the new translation and original rotation
   */
  static frc::geometry::Pose2d
  WithTranslation(frc::geometry::Pose2d pose,
                  frc::geometry::Translation2d translation);

  /**
   * Creates a new pose from an existing one using a different rotation value.
   *
   * @param pose The original pose
   * @param rotation The new rotation to use
   * @return The new pose with the original translation and new rotation
   */
  static frc::geometry::Pose2d WithRotation(frc::geometry::Pose2d pose,
                                            frc::geometry::Rotation2d rotation);
};

} // namespace util