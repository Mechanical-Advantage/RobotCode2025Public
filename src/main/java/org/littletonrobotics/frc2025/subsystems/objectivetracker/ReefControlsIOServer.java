// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.objectivetracker;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Paths;

public class ReefControlsIOServer implements ReefControlsIO {
  private static final String toRobotTable = "/ReefControls/ToRobot";
  private static final String toDashboardTable = "/ReefControls/ToDashboard";
  private static final String selectedLevelTopicName = "SelectedLevel";
  private static final String l1TopicName = "Level1";
  private static final String l2TopicName = "Level2";
  private static final String l3TopicName = "Level3";
  private static final String l4TopicName = "Level4";
  private static final String algaeTopicName = "Algae";
  private static final String coopTopicName = "Coop";
  private static final String isElimsTopicName = "IsElims";

  private final IntegerSubscriber selectedLevelIn;
  private final IntegerSubscriber l1StateIn;
  private final IntegerSubscriber l2StateIn;
  private final IntegerSubscriber l3StateIn;
  private final IntegerSubscriber l4StateIn;
  private final IntegerSubscriber algaeStateIn;
  private final BooleanSubscriber coopStateIn;

  private final IntegerPublisher selectedLevelOut;
  private final IntegerPublisher l1StateOut;
  private final IntegerPublisher l2StateOut;
  private final IntegerPublisher l3StateOut;
  private final IntegerPublisher l4StateOut;
  private final IntegerPublisher algaeStateOut;
  private final BooleanPublisher coopStateOut;
  private final BooleanPublisher isElimsOut;

  public ReefControlsIOServer() {
    // Create subscribers
    var inputTable = NetworkTableInstance.getDefault().getTable(toRobotTable);
    selectedLevelIn =
        inputTable
            .getIntegerTopic(selectedLevelTopicName)
            .subscribe(0, PubSubOption.keepDuplicates(true));
    l1StateIn =
        inputTable.getIntegerTopic(l1TopicName).subscribe(0, PubSubOption.keepDuplicates(true));
    l2StateIn =
        inputTable.getIntegerTopic(l2TopicName).subscribe(0, PubSubOption.keepDuplicates(true));
    l3StateIn =
        inputTable.getIntegerTopic(l3TopicName).subscribe(0, PubSubOption.keepDuplicates(true));
    l4StateIn =
        inputTable.getIntegerTopic(l4TopicName).subscribe(0, PubSubOption.keepDuplicates(true));
    algaeStateIn =
        inputTable.getIntegerTopic(algaeTopicName).subscribe(0, PubSubOption.keepDuplicates(true));
    coopStateIn =
        inputTable
            .getBooleanTopic(coopTopicName)
            .subscribe(false, PubSubOption.keepDuplicates(true));

    // Create publishers
    var outputTable = NetworkTableInstance.getDefault().getTable(toDashboardTable);
    selectedLevelOut = outputTable.getIntegerTopic(selectedLevelTopicName).publish();
    l1StateOut = outputTable.getIntegerTopic(l1TopicName).publish();
    l2StateOut = outputTable.getIntegerTopic(l2TopicName).publish();
    l3StateOut = outputTable.getIntegerTopic(l3TopicName).publish();
    l4StateOut = outputTable.getIntegerTopic(l4TopicName).publish();
    algaeStateOut = outputTable.getIntegerTopic(algaeTopicName).publish();
    coopStateOut = outputTable.getBooleanTopic(coopTopicName).publish();
    isElimsOut = outputTable.getBooleanTopic(isElimsTopicName).publish();

    // Start web server
    WebServer.start(
        5801,
        Paths.get(Filesystem.getDeployDirectory().getAbsolutePath().toString(), "reefcontrols")
            .toString());
  }

  @Override
  public void updateInputs(ReefControlsIOInputs inputs) {
    inputs.selectedLevel =
        selectedLevelIn.readQueue().length > 0
            ? new int[] {(int) selectedLevelIn.get()}
            : new int[] {};
    inputs.level1State =
        l1StateIn.readQueue().length > 0 ? new int[] {(int) l1StateIn.get()} : new int[] {};
    inputs.level2State =
        l2StateIn.readQueue().length > 0 ? new int[] {(int) l2StateIn.get()} : new int[] {};
    inputs.level3State =
        l3StateIn.readQueue().length > 0 ? new int[] {(int) l3StateIn.get()} : new int[] {};
    inputs.level4State =
        l4StateIn.readQueue().length > 0 ? new int[] {(int) l4StateIn.get()} : new int[] {};
    inputs.algaeState =
        algaeStateIn.readQueue().length > 0 ? new int[] {(int) algaeStateIn.get()} : new int[] {};
    inputs.coopState =
        coopStateIn.readQueue().length > 0 ? new boolean[] {coopStateIn.get()} : new boolean[] {};
  }

  @Override
  public void setSelectedLevel(int value) {
    selectedLevelOut.set(value);
  }

  @Override
  public void setLevel1State(int value) {
    l1StateOut.set(value);
  }

  @Override
  public void setLevel2State(int value) {
    l2StateOut.set(value);
  }

  @Override
  public void setLevel3State(int value) {
    l3StateOut.set(value);
  }

  @Override
  public void setLevel4State(int value) {
    l4StateOut.set(value);
  }

  @Override
  public void setAlgaeState(int value) {
    algaeStateOut.set(value);
  }

  @Override
  public void setCoopState(boolean value) {
    coopStateOut.set(value);
  }

  @Override
  public void setElims(boolean isElims) {
    isElimsOut.set(isElims);
  }
}
