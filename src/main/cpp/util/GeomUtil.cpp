// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/GeomUtil.h"

namespace util {

frc::geometry::Transform2d
GeomUtil::ToTransform2d(frc::geometry::Translation2d translation) {
  return frc::geometry::Transform2d(translation, frc::geometry::Rotation2d());
}

frc::geometry::Transform2d GeomUtil::ToTransform2d(double x, double y) {
  return frc::geometry::Transform2d(x, y, frc::geometry::Rotation2d());
}

frc::geometry::Transform2d
GeomUtil::ToTransform2d(frc::geometry::Rotation2d rotation) {
  return frc::geometry::Transform2d(frc::geometry::Translation2d(), rotation);
}

frc::geometry::Transform2d GeomUtil::ToTransform2d(frc::geometry::Pose2d pose) {
  return frc::geometry::Transform2d(pose.Translation(), pose.Rotation());
}

frc::geometry::Pose2d GeomUtil::Inverse(frc::geometry::Pose2d pose) {
  frc::geometry::Rotation2d rotationInverse = pose.Rotation().UnaryMinus();
  return frc::geometry::Pose2d(
      pose.Translation().UnaryMinus().RotateBy(rotationInverse),
      rotationInverse);
}

frc::geometry::Pose2d GeomUtil::ToPose2d(frc::geometry::Transform2d transform) {
  return frc::geometry::Pose2d(transform.Translation(), transform.Rotation());
}

frc::geometry::Pose2d
GeomUtil::ToPose2d(frc::geometry::Translation2d translation) {
  return frc::geometry::Pose2d(translation, frc::geometry::Rotation2d());
}

frc::geometry::Pose2d GeomUtil::ToPose2d(frc::geometry::Rotation2d rotation) {
  return frc::geometry::Pose2d(frc::geometry::Translation2d(), rotation);
}

frc::geometry::Twist2d GeomUtil::Multiply(frc::geometry::Twist2d twist,
                                          double factor) {
  return frc::geometry::Twist2d(twist.dx * factor, twist.dy * factor,
                                twist.dtheta * factor);
}

frc::geometry::Transform3d GeomUtil::ToTransform3d(frc::geometry::Pose3d pose) {
  return frc::geometry::Transform3d(pose.Translation(), pose.Rotation());
}

frc::geometry::Pose3d GeomUtil::ToPose3d(frc::geometry::Transform3d transform) {
  return frc::geometry::Pose3d(transform.Translation(), transform.Rotation());
}

frc::geometry::Twist2d
GeomUtil::ToTwist2d(frc::kinematics::ChassisSpeeds speeds) {
  return frc::geometry::Twist2d(speeds.vx, speeds.vy, speeds.omega);
}

frc::geometry::Pose2d
GeomUtil::WithTranslation(frc::geometry::Pose2d pose,
                          frc::geometry::Translation2d translation) {
  return frc::geometry::Pose2d(translation, pose.Rotation());
}

frc::geometry::Pose2d
GeomUtil::WithRotation(frc::geometry::Pose2d pose,
                       frc::geometry::Rotation2d rotation) {
  return frc::geometry::Pose2d(pose.Translation(), rotation);
}

} // namespace util