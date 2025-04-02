// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.vision;

import static org.littletonrobotics.frc2025.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.*;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.AprilTagLayoutType;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.RobotState.CoralTxTyObservation;
import org.littletonrobotics.frc2025.RobotState.TxTyObservation;
import org.littletonrobotics.frc2025.RobotState.VisionObservation;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.frc2025.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/** Vision subsystem for vision. */
@ExtensionMethod({GeomUtil.class})
public class Vision extends VirtualSubsystem {
  private final Supplier<AprilTagLayoutType> aprilTagLayoutSupplier;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final AprilTagVisionIOInputsAutoLogged[] aprilTagInputs;
  private final ObjDetectVisionIOInputsAutoLogged[] objDetectInputs;
  private final LoggedNetworkBoolean recordingRequest =
      new LoggedNetworkBoolean("/SmartDashboard/Enable Recording", false);

  private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  private final double disconnectedTimeout = 0.5;
  private final Timer[] disconnectedTimers;
  private final Alert[] disconnectedAlerts;

  public Vision(Supplier<AprilTagLayoutType> aprilTagLayoutSupplier, VisionIO... io) {
    this.aprilTagLayoutSupplier = aprilTagLayoutSupplier;
    this.io = io;
    inputs = new VisionIOInputsAutoLogged[io.length];
    aprilTagInputs = new AprilTagVisionIOInputsAutoLogged[io.length];
    objDetectInputs = new ObjDetectVisionIOInputsAutoLogged[io.length];
    disconnectedTimers = new Timer[io.length];
    disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      aprilTagInputs[i] = new AprilTagVisionIOInputsAutoLogged();
      objDetectInputs[i] = new ObjDetectVisionIOInputsAutoLogged();
      disconnectedAlerts[i] = new Alert("", Alert.AlertType.kError);
    }

    // Create map of last frame times for instances
    for (int i = 0; i < io.length; i++) {
      lastFrameTimes.put(i, 0.0);
    }

    for (int i = 0; i < io.length; i++) {
      disconnectedTimers[i] = new Timer();
      disconnectedTimers[i].start();
    }
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i], aprilTagInputs[i], objDetectInputs[i]);
      Logger.processInputs("Vision/Inst" + i, aprilTagInputs[i]);
      Logger.processInputs("AprilTagVision/Inst" + i, aprilTagInputs[i]);
      Logger.processInputs("ObjDetectVision/Inst" + i, objDetectInputs[i]);
    }

    // Update recording state
    boolean shouldRecord = DriverStation.isFMSAttached() || recordingRequest.get();
    for (var ioInst : io) {
      ioInst.setRecording(shouldRecord);
    }

    // Update disconnected alerts & LEDs
    boolean anyNTDisconnected = false;
    for (int i = 0; i < io.length; i++) {
      if (aprilTagInputs[i].timestamps.length > 0 || objDetectInputs[i].timestamps.length > 0) {
        disconnectedTimers[i].reset();
      }
      boolean disconnected =
          disconnectedTimers[i].hasElapsed(disconnectedTimeout) || !inputs[i].ntConnected;
      if (disconnected) {
        disconnectedAlerts[i].setText(
            inputs[i].ntConnected
                ? "Northstar " + i + " connected to NT but not publishing frames"
                : "Northstar " + i + " disconnected from NT");
      }
      disconnectedAlerts[i].set(disconnected);
      anyNTDisconnected = anyNTDisconnected || !inputs[i].ntConnected;
    }
    Leds.getInstance().visionDisconnected = anyNTDisconnected;

    // Loop over instances
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<VisionObservation> allVisionObservations = new ArrayList<>();
    Map<Integer, TxTyObservation> allTxTyObservations = new HashMap<>();
    List<CoralTxTyObservation> allCoralTxTyObservations = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
      // Loop over frames
      for (int frameIndex = 0;
          frameIndex < aprilTagInputs[instanceIndex].timestamps.length;
          frameIndex++) {
        lastFrameTimes.put(instanceIndex, Timer.getTimestamp());
        var timestamp =
            aprilTagInputs[instanceIndex].timestamps[frameIndex] + timestampOffset.get();
        var values = aprilTagInputs[instanceIndex].frames[frameIndex];

        // Exit if blank frame
        if (values.length == 0 || values[0] == 0) {
          continue;
        }

        // Switch based on number of poses
        Pose3d cameraPose = null;
        Pose2d robotPose = null;
        boolean useVisionRotation = false;
        switch ((int) values[0]) {
          case 1:
            // One pose (multi-tag), use directly
            cameraPose =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            robotPose =
                cameraPose
                    .toPose2d()
                    .transformBy(
                        cameras[instanceIndex].pose().get().toPose2d().toTransform2d().inverse());
            useVisionRotation = true;
            break;
          case 2:
            // Two poses (one tag), disambiguate
            double error0 = values[1];
            double error1 = values[9];
            Pose3d cameraPose0 =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            Pose3d cameraPose1 =
                new Pose3d(
                    values[10],
                    values[11],
                    values[12],
                    new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
            Transform2d cameraToRobot =
                cameras[instanceIndex].pose().get().toPose2d().toTransform2d().inverse();
            Pose2d robotPose0 = cameraPose0.toPose2d().transformBy(cameraToRobot);
            Pose2d robotPose1 = cameraPose1.toPose2d().transformBy(cameraToRobot);

            // Check for ambiguity and select based on estimated rotation
            if (error0 < error1 * ambiguityThreshold || error1 < error0 * ambiguityThreshold) {
              Rotation2d currentRotation = RobotState.getInstance().getRotation();
              Rotation2d visionRotation0 = robotPose0.getRotation();
              Rotation2d visionRotation1 = robotPose1.getRotation();
              if (Math.abs(currentRotation.minus(visionRotation0).getRadians())
                  < Math.abs(currentRotation.minus(visionRotation1).getRadians())) {
                cameraPose = cameraPose0;
                robotPose = robotPose0;
              } else {
                cameraPose = cameraPose1;
                robotPose = robotPose1;
              }
            }
            break;
        }

        // Exit if no data
        if (cameraPose == null || robotPose == null) {
          continue;
        }

        // Exit if robot pose is off the field
        if (robotPose.getX() < -fieldBorderMargin
            || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
            || robotPose.getY() < -fieldBorderMargin
            || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) {
          continue;
        }

        // Get tag poses and update last detection times
        List<Pose3d> tagPoses = new ArrayList<>();
        for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i += 10) {
          int tagId = (int) values[i];
          lastTagDetectionTimes.put(tagId, Timer.getTimestamp());
          Optional<Pose3d> tagPose =
              aprilTagLayoutSupplier.get().getLayout().getTagPose((int) values[i]);
          tagPose.ifPresent(tagPoses::add);
        }
        if (tagPoses.isEmpty()) continue;

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double avgDistance = totalDistance / tagPoses.size();

        // Add observation to list
        double xyStdDev =
            xyStdDevCoefficient
                * Math.pow(avgDistance, 1.2)
                / Math.pow(tagPoses.size(), 2.0)
                * cameras[instanceIndex].stdDevFactor();
        double thetaStdDev =
            useVisionRotation
                ? thetaStdDevCoefficient
                    * Math.pow(avgDistance, 1.2)
                    / Math.pow(tagPoses.size(), 2.0)
                    * cameras[instanceIndex].stdDevFactor()
                : Double.POSITIVE_INFINITY;
        allVisionObservations.add(
            new VisionObservation(
                robotPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        allRobotPoses.add(robotPose);

        // Log data from instance
        if (enableInstanceLogging) {
          Logger.recordOutput(
              "AprilTagVision/Inst" + instanceIndex + "/LatencySecs",
              Timer.getTimestamp() - timestamp);
          Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", robotPose);
          Logger.recordOutput(
              "AprilTagVision/Inst" + instanceIndex + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
        }
      }

      // Get tag tx ty observation data
      Map<Integer, TxTyObservation> txTyObservations = new HashMap<>();
      for (int frameIndex = 0;
          frameIndex < aprilTagInputs[instanceIndex].timestamps.length;
          frameIndex++) {
        var timestamp = aprilTagInputs[instanceIndex].timestamps[frameIndex];
        var values = aprilTagInputs[instanceIndex].frames[frameIndex];
        int tagEstimationDataEndIndex =
            switch ((int) values[0]) {
              default -> 0;
              case 1 -> 8;
              case 2 -> 16;
            };

        for (int index = tagEstimationDataEndIndex + 1; index < values.length; index += 10) {
          double[] tx = new double[4];
          double[] ty = new double[4];
          for (int i = 0; i < 4; i++) {
            tx[i] = values[index + 1 + (2 * i)];
            ty[i] = values[index + 1 + (2 * i) + 1];
          }
          int tagId = (int) values[index];
          double distance = values[index + 9];

          txTyObservations.put(
              tagId, new TxTyObservation(tagId, instanceIndex, tx, ty, distance, timestamp));
        }
      }

      // Save tx ty observation data
      for (var observation : txTyObservations.values()) {
        if (!allTxTyObservations.containsKey(observation.tagId())
            || observation.distance() < allTxTyObservations.get(observation.tagId()).distance()) {
          allTxTyObservations.put(observation.tagId(), observation);
        }
      }

      // Record coral observations
      for (int frameIndex = 0;
          frameIndex < objDetectInputs[instanceIndex].timestamps.length;
          frameIndex++) {
        double[] frame = objDetectInputs[instanceIndex].frames[frameIndex];
        for (int i = 0; i < frame.length; i += 10) {
          if (frame[i + 1] > objDetectConfidenceThreshold) {
            double[] tx = new double[4];
            double[] ty = new double[4];
            for (int z = 0; z < 4; z++) {
              tx[z] = frame[i + 2 + (2 * z)];
              ty[z] = frame[i + 2 + (2 * z) + 1];
            }
            allCoralTxTyObservations.add(
                new CoralTxTyObservation(
                    instanceIndex, tx, ty, objDetectInputs[instanceIndex].timestamps[frameIndex]));
          }
        }
      }

      // If no frames from instances, clear robot pose
      if (enableInstanceLogging && aprilTagInputs[instanceIndex].timestamps.length == 0) {
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", Pose2d.kZero);
      }

      // If no recent frames from instance, clear tag poses
      if (enableInstanceLogging
          && Timer.getTimestamp() - lastFrameTimes.get(instanceIndex) > targetLogTimeSecs) {
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/TagPoses", new Pose3d[] {});
      }
    }

    // Log robot poses
    Logger.recordOutput("AprilTagVision/RobotPoses", allRobotPoses.toArray(Pose2d[]::new));

    // Log tag poses
    List<Pose3d> allTagPoses = new ArrayList<>();
    for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
      if (Timer.getTimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
        aprilTagLayoutSupplier
            .get()
            .getLayout()
            .getTagPose(detectionEntry.getKey())
            .ifPresent(allTagPoses::add);
      }
    }
    Logger.recordOutput("AprilTagVision/TagPoses", allTagPoses.toArray(Pose3d[]::new));

    // Send results to robot state
    allVisionObservations.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(RobotState.getInstance()::addVisionObservation);
    allTxTyObservations.values().stream().forEach(RobotState.getInstance()::addTxTyObservation);
    allCoralTxTyObservations.stream()
        .sorted(Comparator.comparingDouble(CoralTxTyObservation::timestamp))
        .forEach(RobotState.getInstance()::addCoralTxTyObservation);

    // Record cycle time
    LoggedTracer.record("Vision");
  }
}
