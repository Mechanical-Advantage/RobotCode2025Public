// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.vision;

import static org.littletonrobotics.frc2025.subsystems.vision.VisionConstants.*;

import edu.wpi.first.networktables.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.littletonrobotics.frc2025.FieldConstants;
import org.littletonrobotics.frc2025.FieldConstants.AprilTagLayoutType;
import org.littletonrobotics.frc2025.util.SystemTimeValidReader;

public class VisionIONorthstar implements VisionIO {
  private final Supplier<AprilTagLayoutType> aprilTagLayoutSupplier;
  private AprilTagLayoutType lastAprilTagLayout = null;

  private final String deviceId;
  private final DoubleArraySubscriber observationSubscriber;
  private final DoubleArraySubscriber objDetectObservationSubscriber;
  private final IntegerSubscriber fpsAprilTagsSubscriber;
  private final IntegerSubscriber fpsObjDetectSubscriber;
  private final StringPublisher eventNamePublisher;
  private final IntegerPublisher matchTypePublisher;
  private final IntegerPublisher matchNumberPublisher;
  private final IntegerPublisher timestampPublisher;
  private final BooleanPublisher isRecordingPublisher;
  private final StringPublisher tagLayoutPublisher;

  private final Timer slowPeriodicTimer = new Timer();

  public VisionIONorthstar(Supplier<AprilTagLayoutType> aprilTagLayoutSupplier, int index) {
    this.aprilTagLayoutSupplier = aprilTagLayoutSupplier;
    this.deviceId = "northstar_" + index;
    var northstarTable = NetworkTableInstance.getDefault().getTable(this.deviceId);
    var configTable = northstarTable.getSubTable("config");
    var camera = cameras[index];

    configTable.getStringTopic("camera_id").publish().set(camera.id());
    configTable.getIntegerTopic("camera_resolution_width").publish().set(camera.width());
    configTable.getIntegerTopic("camera_resolution_height").publish().set(camera.height());
    configTable.getIntegerTopic("camera_auto_exposure").publish().set(camera.autoExposure());
    configTable.getIntegerTopic("camera_exposure").publish().set(camera.exposure());
    configTable.getDoubleTopic("camera_gain").publish().set(camera.gain());
    configTable.getDoubleTopic("camera_denoise").publish().set(camera.denoise());
    configTable.getDoubleTopic("fiducial_size_m").publish().set(FieldConstants.aprilTagWidth);
    isRecordingPublisher = configTable.getBooleanTopic("is_recording").publish();
    isRecordingPublisher.set(false);
    timestampPublisher = configTable.getIntegerTopic("timestamp").publish();
    tagLayoutPublisher = configTable.getStringTopic("tag_layout").publish();
    eventNamePublisher = configTable.getStringTopic("event_name").publish();
    matchTypePublisher = configTable.getIntegerTopic("match_type").publish();
    matchNumberPublisher = configTable.getIntegerTopic("match_number").publish();

    var outputTable = northstarTable.getSubTable("output");
    observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(5),
                PubSubOption.periodic(0.01));
    objDetectObservationSubscriber =
        outputTable
            .getDoubleArrayTopic("objdetect_observations")
            .subscribe(
                new double[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(5),
                PubSubOption.periodic(0.01));
    fpsAprilTagsSubscriber = outputTable.getIntegerTopic("fps_apriltags").subscribe(0);
    fpsObjDetectSubscriber = outputTable.getIntegerTopic("fps_objdetect").subscribe(0);

    slowPeriodicTimer.start();
  }

  public void updateInputs(
      VisionIOInputs inputs,
      AprilTagVisionIOInputs aprilTagInputs,
      ObjDetectVisionIOInputs objDetectInputs) {
    boolean slowPeriodic = slowPeriodicTimer.advanceIfElapsed(1.0);

    // Update NT connection status
    inputs.ntConnected = false;
    for (var client : NetworkTableInstance.getDefault().getConnections()) {
      if (client.remote_id.startsWith(this.deviceId)) {
        inputs.ntConnected = true;
        break;
      }
    }

    // Publish timestamp
    if (slowPeriodic && SystemTimeValidReader.isValid()) {
      timestampPublisher.set(WPIUtilJNI.getSystemTime() / 1000000);
    }

    if (slowPeriodic) {
      eventNamePublisher.set(DriverStation.getEventName());
      matchTypePublisher.set(DriverStation.getMatchType().ordinal());
      matchNumberPublisher.set(DriverStation.getMatchNumber());
    }

    // Publish tag layout
    var aprilTagType = aprilTagLayoutSupplier.get();
    if (aprilTagType != lastAprilTagLayout) {
      lastAprilTagLayout = aprilTagType;
      tagLayoutPublisher.set(aprilTagType.getLayoutString());
    }

    // Get AprilTag data
    var aprilTagQueue = observationSubscriber.readQueue();
    aprilTagInputs.timestamps = new double[aprilTagQueue.length];
    aprilTagInputs.frames = new double[aprilTagQueue.length][];
    for (int i = 0; i < aprilTagQueue.length; i++) {
      aprilTagInputs.timestamps[i] = aprilTagQueue[i].timestamp / 1000000.0;
      aprilTagInputs.frames[i] = aprilTagQueue[i].value;
    }
    if (slowPeriodic) {
      aprilTagInputs.fps = fpsAprilTagsSubscriber.get();
    }

    // Get object detection data
    var objDetectQueue = objDetectObservationSubscriber.readQueue();
    objDetectInputs.timestamps = new double[objDetectQueue.length];
    objDetectInputs.frames = new double[objDetectQueue.length][];
    for (int i = 0; i < objDetectQueue.length; i++) {
      objDetectInputs.timestamps[i] = objDetectQueue[i].timestamp / 1000000.0;
      objDetectInputs.frames[i] = objDetectQueue[i].value;
    }
    if (slowPeriodic) {
      objDetectInputs.fps = fpsObjDetectSubscriber.get();
    }
  }

  public void setRecording(boolean active) {
    isRecordingPublisher.set(active);
  }
}
