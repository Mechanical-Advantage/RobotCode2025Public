// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.*;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2025.Constants.Mode;
import org.littletonrobotics.frc2025.FieldConstants.AprilTagLayoutType;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.commands.*;
import org.littletonrobotics.frc2025.commands.auto.AutoBuilder;
import org.littletonrobotics.frc2025.subsystems.climber.Climber;
import org.littletonrobotics.frc2025.subsystems.climber.Climber.ClimbState;
import org.littletonrobotics.frc2025.subsystems.climber.ClimberIO;
import org.littletonrobotics.frc2025.subsystems.climber.ClimberIOSim;
import org.littletonrobotics.frc2025.subsystems.climber.ClimberIOTalonFX;
import org.littletonrobotics.frc2025.subsystems.drive.*;
import org.littletonrobotics.frc2025.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.subsystems.objectivetracker.ObjectiveTracker;
import org.littletonrobotics.frc2025.subsystems.objectivetracker.ReefControlsIO;
import org.littletonrobotics.frc2025.subsystems.objectivetracker.ReefControlsIOServer;
import org.littletonrobotics.frc2025.subsystems.rollers.*;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.*;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.Elevator;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.ElevatorIO;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.ElevatorIOSim;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import org.littletonrobotics.frc2025.subsystems.superstructure.sensors.CoralSensorIO;
import org.littletonrobotics.frc2025.subsystems.superstructure.sensors.CoralSensorIOLaserCan;
import org.littletonrobotics.frc2025.subsystems.vision.Vision;
import org.littletonrobotics.frc2025.subsystems.vision.VisionIO;
import org.littletonrobotics.frc2025.subsystems.vision.VisionIONorthstar;
import org.littletonrobotics.frc2025.util.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

@ExtensionMethod({DoublePressTracker.class, TriggerUtil.class})
public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;
  private final Superstructure superstructure;
  private RollerSystem funnel;
  private Climber climber;
  private ObjectiveTracker objectiveTracker;
  private final Leds leds = Leds.getInstance();

  // Controllers
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final OverrideSwitches overrides = new OverrideSwitches(5);
  private final Trigger robotRelative = overrides.driverSwitch(0);
  private final Trigger superstructureDisable = overrides.driverSwitch(1);
  private final Trigger superstructureCoast = overrides.driverSwitch(2);
  private final Trigger disableAutoCoralStationIntake = overrides.operatorSwitch(0).negate();
  private final Trigger disableReefAutoAlign = overrides.operatorSwitch(1);
  private final Trigger disableCoralStationAutoAlign = overrides.operatorSwitch(2);
  private final Trigger disableAlgaeScoreAutoAlign = overrides.operatorSwitch(3);
  private final Trigger disableDispenserGamePieceDetection = overrides.operatorSwitch(4);

  private final Trigger aprilTagsReef = overrides.multiDirectionSwitchLeft();
  private final Trigger aprilTagFieldBorder = overrides.multiDirectionSwitchRight();
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.kInfo);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", AlertType.kInfo);
  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

  private boolean superstructureCoastOverride = false;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Elevator elevator = null;
    Dispenser dispenser = null;

    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case COMPBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[0]),
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[1]),
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[2]),
                  new ModuleIOComp(DriveConstants.moduleConfigsComp[3]));
          vision =
              new Vision(
                  this::getSelectedAprilTagLayout,
                  new VisionIONorthstar(this::getSelectedAprilTagLayout, 0),
                  new VisionIONorthstar(this::getSelectedAprilTagLayout, 1));
          elevator = new Elevator(new ElevatorIOTalonFX());
          dispenser =
              new Dispenser(
                  new PivotIOTalonFX(),
                  new RollerSystemIOTalonFX(6, "", 40, true, false, 3.0),
                  new RollerSystemIOTalonFX(7, "", 40, true, false, (30 / 12) * (48 / 18)),
                  new CoralSensorIOLaserCan());
          funnel =
              new RollerSystem("Funnel", new RollerSystemIOTalonFX(2, "", 30, true, false, 1.0));
          climber = new Climber(new ClimberIOTalonFX());
        }
        case DEVBOT -> {
          drive =
              new Drive(
                  new GyroIORedux(),
                  new ModuleIODev(DriveConstants.moduleConfigsDev[0]),
                  new ModuleIODev(DriveConstants.moduleConfigsDev[1]),
                  new ModuleIODev(DriveConstants.moduleConfigsDev[2]),
                  new ModuleIODev(DriveConstants.moduleConfigsDev[3]));
          vision =
              new Vision(
                  this::getSelectedAprilTagLayout,
                  new VisionIONorthstar(this::getSelectedAprilTagLayout, 0),
                  new VisionIONorthstar(this::getSelectedAprilTagLayout, 1));
          elevator = new Elevator(new ElevatorIOTalonFX());
          dispenser =
              new Dispenser(
                  new PivotIO() {},
                  new RollerSystemIOSpark(5, true),
                  new RollerSystemIO() {},
                  new CoralSensorIO() {});
          funnel = new RollerSystem("Funnel", new RollerSystemIOSpark(4, false));
        }
        case SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          elevator = new Elevator(new ElevatorIOSim());
          dispenser =
              new Dispenser(
                  new PivotIOSim(),
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.2),
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.2),
                  new CoralSensorIO() {});
          climber = new Climber(new ClimberIOSim());
          funnel =
              new RollerSystem(
                  "Funnel", new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.02));
        }
      }
    }

    // No-op implementations for replay
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (vision == null) {
      switch (Constants.getRobot()) {
        case COMPBOT ->
            vision =
                new Vision(this::getSelectedAprilTagLayout, new VisionIO() {}, new VisionIO() {});
        case DEVBOT -> vision = new Vision(this::getSelectedAprilTagLayout, new VisionIO() {});
        default -> vision = new Vision(this::getSelectedAprilTagLayout);
      }
    }
    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }
    if (dispenser == null) {
      dispenser =
          new Dispenser(
              new PivotIO() {},
              new RollerSystemIO() {},
              new RollerSystemIO() {},
              new CoralSensorIO() {});
    }
    if (funnel == null) {
      funnel = new RollerSystem("Funnel", new RollerSystemIO() {});
    }
    if (climber == null) {
      climber = new Climber(new ClimberIO() {});
    }
    objectiveTracker =
        new ObjectiveTracker(
            Constants.getMode() == Mode.REPLAY
                ? new ReefControlsIO() {}
                : new ReefControlsIOServer());
    superstructure = new Superstructure(elevator, dispenser);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    LoggedDashboardChooser<Boolean> mirror = new LoggedDashboardChooser<>("Processor Side?");
    mirror.addDefaultOption("Yes", false);
    mirror.addOption("No", true);
    MirrorUtil.setMirror(mirror::get);

    // Set up characterization routines
    var autoBuilder = new AutoBuilder(drive, superstructure, funnel);
    autoChooser.addDefaultOption("Dead In The Water Auto", Commands.none());
    // autoChooser.addOption(
    //     "Super Up In The Water Auto", autoBuilder.superUpInTheWaterAuto());
    // autoChooser.addOption("Up In The Water Auto", autoBuilder.upInTheWaterAuto());
    autoChooser.addOption("Up In The Weeds Auto", autoBuilder.upInTheWeedsAuto(false));
    autoChooser.addOption("Up In The Elimination Auto", autoBuilder.upInTheWeedsAuto(true));
    autoChooser.addOption("Up In The Simplicity Auto", autoBuilder.upInTheSimplicityAuto());
    autoChooser.addOption("Up In The Inspirational Auto", autoBuilder.upInTheInspirationalAuto());
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    HolonomicTrajectory testTrajectory = new HolonomicTrajectory("driveStraight");
    autoChooser.addOption(
        "Drive Trajectory",
        Commands.runOnce(
                () ->
                    RobotState.getInstance()
                        .resetPose(AllianceFlipUtil.apply(testTrajectory.getStartPose())))
            .andThen(new DriveTrajectory(drive, testTrajectory)));
    autoChooser.addOption(
        "Elevator Static Up",
        superstructure.setCharacterizationMode().andThen(elevator.upStaticCharacterization()));
    autoChooser.addOption(
        "Elevator Static Down",
        superstructure.setCharacterizationMode().andThen(elevator.downStaticCharacterization()));
    autoChooser.addOption(
        "Pivot static",
        superstructure.setCharacterizationMode().andThen(dispenser.staticCharacterization()));

    // Set up overrides
    superstructure.setOverrides(superstructureDisable, disableAutoCoralStationIntake);
    elevator.setOverrides(() -> superstructureCoastOverride, superstructureDisable);
    dispenser.setOverrides(
        () -> superstructureCoastOverride,
        superstructureDisable,
        disableDispenserGamePieceDetection);
    climber.setCoastOverride(() -> superstructureCoastOverride);
    objectiveTracker.setForceReefBlocked(superstructure::hasAlgae);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive suppliers
    DoubleSupplier driverX = () -> -driver.getLeftY() - operator.getLeftY();
    DoubleSupplier driverY = () -> -driver.getLeftX() - operator.getLeftX();
    DoubleSupplier driverOmega = () -> -driver.getRightX() - operator.getRightX();

    // Joystick drive command (driver and operator)
    Supplier<Command> joystickDriveCommandFactory =
        () -> DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega, robotRelative);
    drive.setDefaultCommand(joystickDriveCommandFactory.get());

    // ***** DRIVER CONTROLLER *****

    // Auto score coral
    BiConsumer<Trigger, Boolean> bindAutoScore =
        (trigger, firstPriority) -> {
          Container<ReefLevel> lockedReefLevel = new Container<>();
          Container<Boolean> running = new Container<>(false);
          Supplier<Optional<ReefLevel>> levelSupplier =
              firstPriority ? objectiveTracker::getFirstLevel : objectiveTracker::getSecondLevel;
          Trigger blocked =
              new Trigger(
                  () ->
                      levelSupplier
                          .get()
                          .filter(
                              reefLevel ->
                                  objectiveTracker.getCoralObjective(reefLevel).isPresent())
                          .isEmpty());
          trigger
              .and(blocked.negate())
              .onTrue(Commands.runOnce(() -> lockedReefLevel.value = levelSupplier.get().get()))
              .whileTrue(
                  AutoScoreCommands.autoScore(
                          drive,
                          superstructure,
                          funnel,
                          () -> lockedReefLevel.value,
                          () -> objectiveTracker.getCoralObjective(lockedReefLevel.value),
                          driverX,
                          driverY,
                          driverOmega,
                          joystickDriveCommandFactory.get(),
                          disableReefAutoAlign,
                          driver.b())
                      .deadlineFor(
                          Commands.startEnd(
                              () -> running.value = true, () -> running.value = false))
                      .withName("Auto Score Priority #" + (firstPriority ? 1 : 2)));
          trigger
              .and(blocked)
              .and(() -> !running.value)
              .onTrue(
                  Commands.sequence(
                      controllerRumbleCommand().withTimeout(0.1),
                      Commands.waitSeconds(0.1),
                      controllerRumbleCommand().withTimeout(0.1)));
        };
    // Score coral #1
    bindAutoScore.accept(driver.rightTrigger(), true);
    // Score coral #2
    bindAutoScore.accept(driver.rightBumper(), false);

    // Climbing controls
    Container<ClimbState> climbState = new Container<>(ClimbState.START);
    driver
        .y()
        .and(() -> climbState.value == ClimbState.START)
        .doublePress()
        .onTrue(
            climber
                .deploy()
                .andThen(
                    () -> {
                      climbState.value = ClimbState.DEPLOYED;
                      Leds.getInstance().ready = true;
                    })
                .withName("Deploy Climber"));
    driver
        .y()
        .and(() -> climbState.value == ClimbState.DEPLOYED)
        .doublePress()
        .onTrue(
            climber
                .climb()
                .alongWith(Commands.runOnce(() -> climbState.value = ClimbState.CLIMBING))
                .finallyDo(() -> Leds.getInstance().ready = false)
                .withName("Climb Climber"));
    driver
        .y()
        .and(() -> climbState.value == ClimbState.CLIMBING)
        .doublePress()
        .onTrue(
            climber
                .undeploy()
                .andThen(
                    () -> {
                      climbState.value = ClimbState.DEPLOYED;
                      Leds.getInstance().ready = true;
                    })
                .withName("Undeploy Climber"));
    RobotModeTriggers.disabled()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      climbState.value = ClimbState.START;
                      Leds.getInstance().ready = false;
                    })
                .ignoringDisable(true));

    // Coral intake
    driver
        .leftTrigger()
        .whileTrueContinuous(
            Commands.either(
                    joystickDriveCommandFactory.get(),
                    new DriveToStation(drive, driverX, driverY, driverOmega, false),
                    disableCoralStationAutoAlign)
                .deadlineFor(
                    Commands.startEnd(
                        () -> Leds.getInstance().autoScoring = true,
                        () -> Leds.getInstance().autoScoring = false))
                .alongWith(
                    IntakeCommands.intake(superstructure, funnel),
                    Commands.runOnce(superstructure::resetHasCoral))
                .withName("Coral Station Intake"));

    // Algae reef intake & score
    Trigger onOpposingSide =
        new Trigger(
            () ->
                AllianceFlipUtil.applyX(RobotState.getInstance().getEstimatedPose().getX())
                    > FieldConstants.fieldLength / 2);
    Trigger shouldProcess =
        new Trigger(
            () ->
                AllianceFlipUtil.applyY(RobotState.getInstance().getEstimatedPose().getY())
                        < FieldConstants.fieldWidth / 2
                    || onOpposingSide.getAsBoolean());
    Container<Boolean> hasAlgae = new Container<>(false);
    driver.leftBumper().onTrue(Commands.runOnce(() -> hasAlgae.value = superstructure.hasAlgae()));

    // Algae reef intake
    driver
        .leftBumper()
        .and(() -> !hasAlgae.value)
        .whileTrue(
            AutoScoreCommands.reefIntake(
                    drive,
                    superstructure,
                    objectiveTracker::getAlgaeObjective,
                    driverX,
                    driverY,
                    driverOmega,
                    joystickDriveCommandFactory.get(),
                    disableReefAutoAlign)
                .deadlineFor(
                    Commands.startEnd(
                        () -> Leds.getInstance().autoScoring = true,
                        () -> Leds.getInstance().autoScoring = false))
                .onlyIf(() -> objectiveTracker.getAlgaeObjective().isPresent())
                .withName("Algae Reef Intake"));

    // Algae pre-processor
    driver
        .leftBumper()
        .and(shouldProcess)
        .and(() -> hasAlgae.value)
        .and(driver.a().negate())
        .whileTrueContinuous(
            AlgaeScoreCommands.process(
                    drive,
                    superstructure,
                    driverX,
                    driverY,
                    driverOmega,
                    joystickDriveCommandFactory.get(),
                    onOpposingSide,
                    false,
                    disableAlgaeScoreAutoAlign)
                .withName("Algae Pre-Processor"));

    // Algae process
    driver
        .leftBumper()
        .and(shouldProcess)
        .and(() -> hasAlgae.value)
        .and(driver.a())
        .whileTrueContinuous(
            AlgaeScoreCommands.process(
                    drive,
                    superstructure,
                    driverX,
                    driverY,
                    driverOmega,
                    joystickDriveCommandFactory.get(),
                    onOpposingSide,
                    true,
                    disableAlgaeScoreAutoAlign)
                .withName("Algae Processing"));

    // Algae pre-net
    driver
        .leftBumper()
        .and(shouldProcess.negate())
        .and(() -> hasAlgae.value)
        .and(driver.a().negate())
        .whileTrue(
            AlgaeScoreCommands.netThrowLineup(
                    drive,
                    superstructure,
                    driverY,
                    joystickDriveCommandFactory.get(),
                    disableAlgaeScoreAutoAlign)
                .withName("Algae Pre-Net"))
        .onTrue(Commands.runOnce(() -> Leds.getInstance().autoScoring = true))
        .onFalse(Commands.runOnce(() -> Leds.getInstance().autoScoring = false))
        // Indicate ready for score
        .and(() -> superstructure.getState() == SuperstructureState.PRE_THROWN)
        .whileTrue(
            controllerRumbleCommand()
                .withTimeout(0.1)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly())
        .onTrue(Commands.runOnce(() -> Leds.getInstance().ready = true))
        .onFalse(Commands.runOnce(() -> Leds.getInstance().ready = false));

    // Algae net score
    driver
        .leftBumper()
        .and(shouldProcess.negate())
        .and(() -> hasAlgae.value)
        .and(driver.a())
        .whileTrue(
            AlgaeScoreCommands.netThrowScore(drive, superstructure).withName("Algae Net Score"));

    // Algae eject
    driver
        .a()
        .and(driver.leftBumper().negate())
        .whileTrue(superstructure.runGoal(SuperstructureState.TOSS).withName("Algae Toss"));

    // Strobe LEDs at human player
    driver
        .x()
        .whileTrue(
            Commands.startEnd(
                    () -> leds.hpAttentionAlert = true, () -> leds.hpAttentionAlert = false)
                .withName("Strobe LEDs at HP"));

    // Coral eject
    driver
        .b()
        .and(driver.rightBumper().negate())
        .and(driver.rightTrigger().negate())
        .whileTrue(
            superstructure
                .runGoal(SuperstructureState.GOODBYE_CORAL_EJECT)
                .alongWith(funnel.runRoller(IntakeCommands.outtakeVolts))
                .withName("Coral Eject"));

    // Force net
    driver
        .povLeft()
        .whileTrue(superstructure.runGoal(SuperstructureState.THROWN).withName("Force Net"));

    // Force processor
    driver
        .povRight()
        .whileTrue(
            superstructure.runGoal(SuperstructureState.PROCESSED).withName("Force Processor"));

    // Raise elevator
    driver
        .povUp()
        .toggleOnTrue(
            superstructure.runGoal(SuperstructureState.L2_CORAL).withName("Force Raise Elevator"));

    // ***** OPERATOR CONTROLLER *****

    // Algae stow intake
    operator
        .leftTrigger()
        .whileTrue(
            superstructure
                .runGoal(SuperstructureState.ALGAE_STOW_INTAKE)
                .withName("Algae Stow Intake"));

    // Algae reef intake
    operator
        .povDown()
        .whileTrue(
            superstructure
                .runGoal(SuperstructureState.ALGAE_L2_INTAKE)
                .withName("Operator Algae L2 Intake"));
    operator
        .povUp()
        .whileTrue(
            superstructure
                .runGoal(SuperstructureState.ALGAE_L3_INTAKE)
                .withName("Operator Algae L3 Intake"));

    // Coral intake
    operator
        .rightBumper()
        .whileTrue(
            IntakeCommands.intake(superstructure, funnel)
                .alongWith(Commands.runOnce(superstructure::resetHasCoral))
                .withName("Operator Coral Intake"));

    // Home elevator
    operator
        .leftBumper()
        .onTrue(superstructure.runHomingSequence().withName("Operator Home Elevator"));

    // Force net
    operator
        .povLeft()
        .whileTrue(
            superstructure.runGoal(SuperstructureState.THROWN).withName("Operator Force Net"));

    // Force processor
    operator
        .povRight()
        .whileTrue(
            superstructure
                .runGoal(SuperstructureState.PROCESSED)
                .withName("Operator Force Processor"));

    // Algae eject
    operator
        .rightTrigger()
        .and(operator.a().negate())
        .and(operator.b().negate())
        .and(operator.x().negate())
        .and(operator.y().negate())
        .whileTrue(
            superstructure.runGoal(SuperstructureState.TOSS).withName("Operator Algae Eject"));

    // Operator commands for superstructure
    BiConsumer<Trigger, ReefLevel> bindOperatorCoralScore =
        (faceButton, height) -> {
          faceButton.whileTrueContinuous(
              superstructure
                  .runGoal(() -> Superstructure.getScoringState(height, false))
                  .withName("Operator Score on " + height));
          faceButton
              .and(operator.rightTrigger())
              .whileTrueContinuous(
                  superstructure
                      .runGoal(() -> Superstructure.getScoringState(height, true))
                      .withName("Operator Score & Eject On " + height));
        };
    bindOperatorCoralScore.accept(operator.a(), ReefLevel.L1);
    bindOperatorCoralScore.accept(operator.x(), ReefLevel.L2);
    bindOperatorCoralScore.accept(operator.b(), ReefLevel.L3);
    bindOperatorCoralScore.accept(operator.y(), ReefLevel.L4);

    // ***** MISCELlANEOUS *****

    // Auto intake coral
    funnel.setDefaultCommand(
        funnel
            .runRoller(
                () -> {
                  if (superstructure.isRequestFunnelIntake()) {
                    return IntakeCommands.funnelVolts.get();
                  } else if (superstructure.isRequestFunnelOuttake()) {
                    return IntakeCommands.outtakeVolts.get();
                  } else {
                    return 0.0;
                  }
                })
            .withName("Auto Intake Coral"));

    // Reset gyro
    var driverStartAndBack = driver.start().and(driver.back());
    var operatorStartAndBack = operator.start().and(operator.back());
    driverStartAndBack
        .or(operatorStartAndBack)
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .resetPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                                    AllianceFlipUtil.apply(Rotation2d.kZero))))
                .withName("Reset Gyro")
                .ignoringDisable(true));

    // Superstructure coast
    superstructureCoast
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (DriverStation.isDisabled()) {
                        superstructureCoastOverride = true;
                        leds.superstructureCoast = true;
                      }
                    })
                .withName("Superstructure Coast")
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(
                    () -> {
                      superstructureCoastOverride = false;
                      leds.superstructureCoast = false;
                    })
                .withName("Superstructure Uncoast")
                .ignoringDisable(true));
    RobotModeTriggers.disabled()
        .onFalse(
            Commands.runOnce(
                    () -> {
                      superstructureCoastOverride = false;
                      leds.superstructureCoast = false;
                    })
                .ignoringDisable(true));

    // Endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.5)
                .beforeStarting(() -> leds.endgameAlert = true)
                .finallyDo(() -> leds.endgameAlert = false)
                .withName("Controller Endgame Alert 1"));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(0.9)
                .beforeStarting(() -> leds.endgameAlert = true)
                .finallyDo(() -> leds.endgameAlert = false)
                .withName("Controller Endgame Alert 2")); // Rumble three times
  }

  // Creates controller rumble command
  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  // Update dashboard data
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  public void updateAlerts() {
    // Controller disconnected alerts
    driverDisconnected.set(!DriverStation.isJoystickConnected(driver.getHID().getPort()));
    operatorDisconnected.set(!DriverStation.isJoystickConnected(operator.getHID().getPort()));
    overrideDisconnected.set(!overrides.isConnected());

    // AprilTag layout alert
    boolean aprilTagAlertActive = getSelectedAprilTagLayout() != FieldConstants.defaultAprilTagType;
    aprilTagLayoutAlert.set(aprilTagAlertActive);
    if (aprilTagAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-default AprilTag layout in use (" + getSelectedAprilTagLayout().toString() + ").");
    }
  }

  /** Returns the current AprilTag layout type. */
  public AprilTagLayoutType getSelectedAprilTagLayout() {
    if (aprilTagsReef.getAsBoolean()) {
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
        return FieldConstants.AprilTagLayoutType.BLUE_REEF;
      } else {
        return FieldConstants.AprilTagLayoutType.RED_REEF;
      }
    } else if (aprilTagFieldBorder.getAsBoolean()) {
      return FieldConstants.AprilTagLayoutType.FIELD_BORDER;
    } else {
      return FieldConstants.defaultAprilTagType;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
