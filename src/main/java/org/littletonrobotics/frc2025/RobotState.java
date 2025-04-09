// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.*;
import java.util.stream.Collectors;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.vision.VisionConstants;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class RobotState {
  // Must be less than 2.0
  private static final LoggedTunableNumber txTyObservationStaleSecs =
      new LoggedTunableNumber("RobotState/TxTyObservationStaleSeconds", 0.05);
  private static final LoggedTunableNumber minDistanceTagPoseBlend =
      new LoggedTunableNumber("RobotState/MinDistanceTagPoseBlend", Units.inchesToMeters(24.0));
  private static final LoggedTunableNumber maxDistanceTagPoseBlend =
      new LoggedTunableNumber("RobotState/MaxDistanceTagPoseBlend", Units.inchesToMeters(36.0));
  private static final LoggedTunableNumber coralOverlap =
      new LoggedTunableNumber("RobotState/CoralOverlap", .5);
  private static final LoggedTunableNumber coralPersistanceTime =
      new LoggedTunableNumber("RobotState/CoralPersistanceTime", 2.0);

  private static final double poseBufferSizeSec = 2.0;
  private static final Matrix<N3, N1> odometryStateStdDevs =
      new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));
  private static final Map<Integer, Pose2d> tagPoses2d = new HashMap<>();

  static {
    for (int i = 1; i <= FieldConstants.aprilTagCount; i++) {
      tagPoses2d.put(
          i,
          FieldConstants.defaultAprilTagType
              .getLayout()
              .getTagPose(i)
              .map(Pose3d::toPose2d)
              .orElse(Pose2d.kZero));
    }
  }

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // Pose Estimation Members
  @Getter @AutoLogOutput private Pose2d odometryPose = Pose2d.kZero;
  @Getter @AutoLogOutput private Pose2d estimatedPose = Pose2d.kZero;

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
  // Odometry
  private final SwerveDriveKinematics kinematics;
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  // Assume gyro starts at zero
  private Rotation2d gyroOffset = Rotation2d.kZero;

  private final Map<Integer, TxTyPoseRecord> txTyPoses = new HashMap<>();
  private Set<CoralPoseRecord> coralPoses = new HashSet<>();

  @Getter
  @AutoLogOutput(key = "RobotState/RobotVelocity")
  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  @Getter
  @Setter
  @AutoLogOutput(key = "RobotState/ElevatorExtensionPercent")
  private double elevatorExtensionPercent;

  @Getter @Setter private OptionalDouble distanceToBranch = OptionalDouble.empty();
  @Getter @Setter private OptionalDouble distanceToReefAlgae = OptionalDouble.empty();
  @Getter @Setter private Rotation2d pitch = Rotation2d.kZero;
  @Getter @Setter private Rotation2d roll = Rotation2d.kZero;

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
    }
    kinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);

    for (int i = 1; i <= FieldConstants.aprilTagCount; i++) {
      txTyPoses.put(i, new TxTyPoseRecord(Pose2d.kZero, Double.POSITIVE_INFINITY, -1.0));
    }
  }

  public void resetPose(Pose2d pose) {
    // Gyro offset is the rotation that maps the old gyro rotation (estimated - offset) to the new
    // frame of rotation
    gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
    estimatedPose = pose;
    odometryPose = pose;
    poseBuffer.clear();
  }

  public void addOdometryObservation(OdometryObservation observation) {
    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();
    Pose2d lastOdometryPose = odometryPose;
    odometryPose = odometryPose.exp(twist);
    // Use gyro if connected
    observation.gyroAngle.ifPresent(
        gyroAngle -> {
          // Add offset to measured angle
          Rotation2d angle = gyroAngle.plus(gyroOffset);
          odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
        });
    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);
    // Calculate diff from last odometry pose and add onto pose estimate
    Twist2d finalTwist = lastOdometryPose.log(odometryPose);
    estimatedPose = estimatedPose.exp(finalTwist);
  }

  public void addVisionObservation(VisionObservation observation) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSec > observation.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }
    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    // sample --> odometryPose transform and backwards of that
    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
    // get old estimate by applying odometryToSample Transform
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    }
    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }
    // difference between estimate and vision pose
    Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
    // scale transform by visionK
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate current estimate by applying scaled transform to old estimate
    // then replaying odometry data
    estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  }

  public void addTxTyObservation(TxTyObservation observation) {
    // Skip if current data for tag is newer
    if (txTyPoses.containsKey(observation.tagId())
        && txTyPoses.get(observation.tagId()).timestamp() >= observation.timestamp()) {
      return;
    }

    // Get rotation at timestamp
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }
    Rotation2d robotRotation =
        estimatedPose.transformBy(new Transform2d(odometryPose, sample.get())).getRotation();

    // Average tx's and ty's
    double tx = 0.0;
    double ty = 0.0;
    for (int i = 0; i < 4; i++) {
      tx += observation.tx()[i];
      ty += observation.ty()[i];
    }
    tx /= 4.0;
    ty /= 4.0;

    Pose3d cameraPose = VisionConstants.cameras[observation.camera()].pose().get();

    // Use 3D distance and tag angles to find robot pose
    Translation2d camToTagTranslation =
        new Pose3d(Translation3d.kZero, new Rotation3d(0, ty, -tx))
            .transformBy(
                new Transform3d(new Translation3d(observation.distance(), 0, 0), Rotation3d.kZero))
            .getTranslation()
            .rotateBy(new Rotation3d(0, cameraPose.getRotation().getY(), 0))
            .toTranslation2d();
    Rotation2d camToTagRotation =
        robotRotation.plus(
            cameraPose.toPose2d().getRotation().plus(camToTagTranslation.getAngle()));
    var tagPose2d = tagPoses2d.get(observation.tagId());
    if (tagPose2d == null) return;
    Translation2d fieldToCameraTranslation =
        new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
            .transformBy(GeomUtil.toTransform2d(camToTagTranslation.getNorm(), 0.0))
            .getTranslation();
    Pose2d robotPose =
        new Pose2d(
                fieldToCameraTranslation, robotRotation.plus(cameraPose.toPose2d().getRotation()))
            .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));
    // Use gyro angle at time for robot rotation
    robotPose = new Pose2d(robotPose.getTranslation(), robotRotation);

    // Add transform to current odometry based pose for latency correction
    txTyPoses.put(
        observation.tagId(),
        new TxTyPoseRecord(robotPose, camToTagTranslation.getNorm(), observation.timestamp()));
  }

  public void addDriveSpeeds(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  /** Get 2d pose estimate of robot if not stale. */
  public Optional<Pose2d> getTxTyPose(int tagId) {
    if (!txTyPoses.containsKey(tagId)) {
      return Optional.empty();
    }
    var data = txTyPoses.get(tagId);
    // Check if stale
    if (Timer.getTimestamp() - data.timestamp() >= txTyObservationStaleSecs.get()) {
      return Optional.empty();
    }
    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(data.timestamp());
    // Latency compensate
    return sample.map(pose2d -> data.pose().plus(new Transform2d(pose2d, odometryPose)));
  }

  /**
   * Get estimated pose using txty data given tagId on reef and aligned pose on reef. Used for algae
   * intaking and coral scoring.
   */
  public ReefPoseEstimate getReefPose(int face, Pose2d finalPose) {
    final boolean isRed = AllianceFlipUtil.shouldFlip();
    var tagPose =
        getTxTyPose(
            switch (face) {
              case 1 -> isRed ? 6 : 19;
              case 2 -> isRed ? 11 : 20;
              case 3 -> isRed ? 10 : 21;
              case 4 -> isRed ? 9 : 22;
              case 5 -> isRed ? 8 : 17;
                // 0
              default -> isRed ? 7 : 18;
            });
    // Use estimated pose if tag pose is not present
    if (tagPose.isEmpty()) return new ReefPoseEstimate(getEstimatedPose(), 0.0);
    // Use distance from estimated pose to final pose to get t value
    final double t =
        MathUtil.clamp(
            (getEstimatedPose().getTranslation().getDistance(finalPose.getTranslation())
                    - minDistanceTagPoseBlend.get())
                / (maxDistanceTagPoseBlend.get() - minDistanceTagPoseBlend.get()),
            0.0,
            1.0);
    return new ReefPoseEstimate(getEstimatedPose().interpolate(tagPose.get(), 1.0 - t), 1.0 - t);
  }

  public void addCoralTxTyObservation(CoralTxTyObservation observation) {
    var oldOdometryPose = poseBuffer.getSample(observation.timestamp());
    if (oldOdometryPose.isEmpty()) {
      return;
    }
    Pose2d fieldToRobot =
        estimatedPose.transformBy(new Transform2d(odometryPose, oldOdometryPose.get()));
    Pose3d robotToCamera = VisionConstants.cameras[observation.camera()].pose().get();

    // Assume coral height of zero and find midpoint of width of bottom tx ty
    double tx = (observation.tx()[2] + observation.tx()[3]) / 2;
    double ty = (observation.ty()[2] + observation.ty()[3]) / 2;

    double cameraToCoralAngle = -robotToCamera.getRotation().getY() - ty;
    if (cameraToCoralAngle >= 0) {
      return;
    }

    double cameraToCoralNorm =
        (-robotToCamera.getZ())
            / Math.tan(-robotToCamera.getRotation().getY() - ty)
            / Math.cos(-tx);
    Pose2d fieldToCamera = fieldToRobot.transformBy(robotToCamera.toPose2d().toTransform2d());
    Pose2d fieldToCoral =
        fieldToCamera
            .transformBy(new Transform2d(Translation2d.kZero, new Rotation2d(-tx)))
            .transformBy(
                new Transform2d(new Translation2d(cameraToCoralNorm, 0), Rotation2d.kZero));
    Translation2d fieldToCoralTranslation2d = fieldToCoral.getTranslation();
    CoralPoseRecord coralPoseRecord =
        new CoralPoseRecord(fieldToCoralTranslation2d, observation.timestamp());

    coralPoses =
        coralPoses.stream()
            .filter(
                (x) -> x.translation.getDistance(fieldToCoralTranslation2d) > coralOverlap.get())
            .collect(Collectors.toSet());
    coralPoses.add(coralPoseRecord);
  }

  public Set<Translation2d> getCoralTranslations() {
    return coralPoses.stream()
        .filter((x) -> Timer.getTimestamp() - x.timestamp() < coralPersistanceTime.get())
        .map(CoralPoseRecord::translation)
        .collect(Collectors.toSet());
  }

  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  public void periodicLog() {
    // Log tx/ty poses
    Pose2d[] tagPoses = new Pose2d[FieldConstants.aprilTagCount + 1];
    for (int i = 0; i < FieldConstants.aprilTagCount + 1; i++) {
      tagPoses[i] = getTxTyPose(i).orElse(Pose2d.kZero);
    }
    Logger.recordOutput("RobotState/TxTyPoses", tagPoses);

    // Log coral poses
    Logger.recordOutput(
        "RobotState/CoralPoses",
        getCoralTranslations().stream()
            .map(
                (translation) ->
                    new Pose3d(
                        new Translation3d(
                            translation.getX(),
                            translation.getY(),
                            FieldConstants.coralDiameter / 2.0),
                        new Rotation3d(new Rotation2d(Timer.getTimestamp() * 5.0))))
            .toArray(Pose3d[]::new));
  }

  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}

  public record TxTyObservation(
      int tagId, int camera, double[] tx, double[] ty, double distance, double timestamp) {}

  public record TxTyPoseRecord(Pose2d pose, double distance, double timestamp) {}

  public record CoralTxTyObservation(int camera, double[] tx, double[] ty, double timestamp) {}

  public record CoralPoseRecord(Translation2d translation, double timestamp) {}

  public record ReefPoseEstimate(Pose2d pose, double blend) {}
}
