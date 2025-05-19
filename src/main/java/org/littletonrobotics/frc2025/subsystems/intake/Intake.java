// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2025.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2025.subsystems.sensors.CoralSensorIO;
import org.littletonrobotics.frc2025.subsystems.sensors.CoralSensorIOInputsAutoLogged;
import org.littletonrobotics.frc2025.util.AutoCoralSim;
import org.littletonrobotics.frc2025.util.GeomUtil;
import org.littletonrobotics.frc2025.util.LoggedTracer;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final Transform3d coralIndexedTransform =
      GeomUtil.toTransform3d(
          new Pose3d(
                  new Translation3d(0.0, 0.0, 0.25),
                  new Rotation3d(0.0, Units.degreesToRadians(-10.0), 0.0))
              .transformBy(
                  GeomUtil.toTransform3d(
                      new Pose3d(-Units.inchesToMeters(11.0) / 2.0, 0.0, 0.0, Rotation3d.kZero))));

  private static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Intake/Intake/Roller", 12.0);
  private static final LoggedTunableNumber indexerIntakeVolts =
      new LoggedTunableNumber("Intake/Intake/Indexer", 12.0);
  private static final LoggedTunableNumber rollerOuttakeVolts =
      new LoggedTunableNumber("Intake/Outtake/Roller", -8.0);
  private static final LoggedTunableNumber rollerL1EjectVolts =
      new LoggedTunableNumber("Intake/L1Eject/Roller", -2.0);
  private static final LoggedTunableNumber indexerOuttakeVolts =
      new LoggedTunableNumber("Intake/Outtake/Indexer", -6.0);
  private static final LoggedTunableNumber indexerIndexVolts =
      new LoggedTunableNumber("Intake/Index/Indexer", 8.0);
  private static final LoggedTunableNumber rollerReverseVolts =
      new LoggedTunableNumber("Intake/Reverse/Roller", -3.0);
  private static final LoggedTunableNumber indexerReverseVolts =
      new LoggedTunableNumber("Intake/Reverse/Indexer", -3.0);
  private static final LoggedTunableNumber l1ReverseTimeSecs =
      new LoggedTunableNumber("Intake/Reverse/L1ReverseTimeSecs", 0.43);
  private static final LoggedTunableNumber coralProximity =
      new LoggedTunableNumber("Intake/CoralProximity", 0.05);

  private final Slam slam;
  private final RollerSystem roller;
  private final RollerSystem indexer;
  private final CoralSensorIO coralSensorIO;
  private final CoralSensorIOInputsAutoLogged coralSensorInputs =
      new CoralSensorIOInputsAutoLogged();

  @AutoLogOutput private Goal goal = Goal.RETRACT;
  private final Timer goalTimer = new Timer();
  @Getter @AutoLogOutput private boolean coralIndexed;
  @Getter @AutoLogOutput private boolean coralIndexedRaw;
  private final Debouncer coralIndexedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  @Setter private boolean hasCoral;
  @AutoLogOutput @Setter private boolean superstructureReady;
  @Setter private BooleanConsumer coralIndexedConsumer;
  private boolean shouldIndex = true;

  @Setter private BooleanSupplier coastOverride = () -> false;

  public Intake(
      SlamIO slamIO,
      RollerSystemIO rollerIO,
      RollerSystemIO indexerIO,
      CoralSensorIO coralSensorIO) {
    this.slam = new Slam(slamIO);
    this.roller = new RollerSystem("Intake roller", "Intake/Roller", rollerIO);
    this.indexer = new RollerSystem("Intake indexer", "Intake/Indexer", indexerIO);
    this.coralSensorIO = coralSensorIO;
    goalTimer.start();
    roller.setBrakeMode(true);
    indexer.setBrakeMode(true);
  }

  public void periodic() {
    // Set coast mode
    slam.setBrakeMode(!coastOverride.getAsBoolean());

    slam.periodic();
    roller.periodic();
    indexer.periodic();
    coralSensorIO.updateInputs(coralSensorInputs);
    Logger.processInputs("Intake/CoralSensor", coralSensorInputs);

    // Update coral index state
    if (Constants.getRobot() != Constants.RobotType.SIMBOT) {
      coralIndexedRaw =
          coralSensorInputs.data.valid()
              && coralSensorInputs.data.distanceMeters() <= coralProximity.get();
      coralIndexed = coralIndexedDebouncer.calculate(coralIndexedRaw);
    } else if (DriverStation.isAutonomousEnabled()) {
      if (goal == Goal.INTAKE && !coralIndexed && !hasCoral) {
        coralIndexed =
            AutoCoralSim.intakeCoral(
                RobotState.getInstance()
                    .getEstimatedPose()
                    .transformBy(
                        GeomUtil.toTransform2d(
                            -(DriveConstants.robotWidth / 2.0 + Units.inchesToMeters(5.0)), 0.0))
                    .transformBy(GeomUtil.toTransform2d(Rotation2d.kPi)));
      }
      coralIndexed = coralIndexed && !hasCoral;
    }
    coralIndexedConsumer.accept(coralIndexed);

    // Calculate goal
    Slam.Goal slamGoal = null;
    double rollerVolts = 0.0;
    double indexerVolts = 0.0;
    switch (goal) {
      case RETRACT -> {
        slamGoal = Slam.Goal.RETRACT;
        if (slam.getGoal() == Slam.Goal.DEPLOY && slam.atGoal()) {
          if (hasCoral) {
            rollerVolts = rollerOuttakeVolts.get();
            indexerVolts = indexerOuttakeVolts.get();
          } else if (coralIndexed) {
            rollerVolts = rollerOuttakeVolts.get();
          }
        }
      }
      case DEPLOY -> {
        slamGoal = Slam.Goal.DEPLOY;
        if (coralIndexed && hasCoral) {
          indexerVolts = indexerOuttakeVolts.get();
        }
      }
      case INTAKE -> {
        slamGoal = Slam.Goal.DEPLOY;
        if (slam.getGoal() != Slam.Goal.DEPLOY || !slam.atGoal()) break;
        if (hasCoral || coralIndexed) {
          rollerVolts = rollerOuttakeVolts.get();
        } else {
          rollerVolts = rollerIntakeVolts.get();
        }

        if (hasCoral) {
          indexerVolts = indexerOuttakeVolts.get();
        } else {
          if (coralIndexed) {
            indexerVolts = 0.0;
          } else {
            indexerVolts = indexerIntakeVolts.get();
          }
        }
      }
      case OUTTAKE -> {
        slamGoal = Slam.Goal.DEPLOY;
        if (slam.getGoal() != Slam.Goal.DEPLOY || !slam.atGoal()) break;
        rollerVolts = rollerOuttakeVolts.get();
        indexerVolts = indexerOuttakeVolts.get();
      }
      case REVERSE -> {
        slamGoal = Slam.Goal.REVERSE;
        if (slam.getGoal() != Slam.Goal.REVERSE || !slam.atGoal()) break;
        rollerVolts = rollerReverseVolts.get();
        indexerVolts = indexerReverseVolts.get();
      }
      case L1 -> slamGoal = Slam.Goal.L1;
      case L1_EJECT -> {
        slamGoal = Slam.Goal.L1_EJECT;
        if (slam.getGoal() != Slam.Goal.L1_EJECT || !slam.atGoal()) break;
        rollerVolts = rollerL1EjectVolts.get();
      }
      case CLIMB -> {
        slamGoal = Slam.Goal.CLIMB;
        rollerVolts = 0.0;
      }
    }

    // Handle indexing to superstructure
    if (coralIndexed && !hasCoral && superstructureReady && shouldIndex && goal != Goal.OUTTAKE) {
      indexerVolts = indexerIndexVolts.get();
    }

    // Run state
    slam.setGoal(slamGoal);
    roller.setVolts(rollerVolts);
    indexer.setVolts(indexerVolts);

    // Get default goal
    if (DriverStation.isDisabled()) {
      goal = slam.wantsToDeploy() ? Goal.DEPLOY : Goal.RETRACT;
    }

    // Visualize coral indexed
    Logger.recordOutput(
        "Mechanism3d/Measured/CoralIndexed",
        coralIndexed
            ? new Pose3d[] {
              new Pose3d(RobotState.getInstance().getEstimatedPose())
                  .transformBy(coralIndexedTransform)
            }
            : new Pose3d[] {});

    // Record cycle time
    LoggedTracer.record("Intake");
  }

  public Command intake() {
    return startEnd(() -> setGoal(Goal.INTAKE), () -> setGoal(Goal.DEPLOY));
  }

  public Command intakeTeleop() {
    return startEnd(() -> setGoal(Goal.INTAKE), () -> setGoal(Goal.RETRACT));
  }

  public Command outtake() {
    return startEnd(() -> setGoal(Goal.OUTTAKE), () -> setGoal(Goal.DEPLOY));
  }

  public Command retract() {
    return setGoalCommand(Goal.RETRACT);
  }

  public Command deploy() {
    return runOnce(() -> setGoal(Goal.DEPLOY));
  }

  public Command prepareToClimb() {
    return setGoalCommand(Goal.CLIMB);
  }

  public Command l1Sequence(BooleanSupplier eject, BooleanSupplier hasCoral) {
    Timer reverseTimer = new Timer();
    return intake()
        .until(hasCoral)
        .andThen(
            setGoalCommand(Goal.REVERSE),
            Commands.waitUntil(() -> coralIndexedRaw),
            Commands.waitUntil(() -> !coralIndexedRaw),
            Commands.runOnce(reverseTimer::restart),
            setGoalCommand(Goal.REVERSE),
            Commands.waitUntil(() -> reverseTimer.hasElapsed(l1ReverseTimeSecs.get())),
            setGoalCommand(Goal.L1),
            Commands.waitUntil(eject),
            setGoalCommand(Goal.L1_EJECT))
        .deadlineFor(Commands.startEnd(() -> shouldIndex = false, () -> shouldIndex = true));
  }

  private Command setGoalCommand(Goal goal) {
    return runOnce(() -> setGoal(goal));
  }

  private void setGoal(Goal goal) {
    if (this.goal == goal) return;
    this.goal = goal;
    goalTimer.restart();
  }

  public Command runHomingSequence() {
    return Commands.runOnce(slam::overrideHoming);
  }

  public enum Goal {
    RETRACT,
    DEPLOY,
    INTAKE,
    OUTTAKE,
    REVERSE,
    L1,
    L1_EJECT,
    CLIMB
  }
}
