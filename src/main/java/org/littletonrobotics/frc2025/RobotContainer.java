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
import org.littletonrobotics.frc2025.FieldConstants.AlgaeObjective;
import org.littletonrobotics.frc2025.FieldConstants.AprilTagLayoutType;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.commands.*;
import org.littletonrobotics.frc2025.subsystems.climber.Climber;
import org.littletonrobotics.frc2025.subsystems.climber.Climber.ClimbState;
import org.littletonrobotics.frc2025.subsystems.climber.ClimberIO;
import org.littletonrobotics.frc2025.subsystems.climber.ClimberIOSim;
import org.littletonrobotics.frc2025.subsystems.climber.ClimberIOTalonFX;
import org.littletonrobotics.frc2025.subsystems.drive.*;
import org.littletonrobotics.frc2025.subsystems.intake.Intake;
import org.littletonrobotics.frc2025.subsystems.intake.SlamIO;
import org.littletonrobotics.frc2025.subsystems.intake.SlamIOSim;
import org.littletonrobotics.frc2025.subsystems.intake.SlamIOTalonFX;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.subsystems.objectivetracker.ObjectiveTracker;
import org.littletonrobotics.frc2025.subsystems.objectivetracker.ReefControlsIO;
import org.littletonrobotics.frc2025.subsystems.objectivetracker.ReefControlsIOServer;
import org.littletonrobotics.frc2025.subsystems.rollers.*;
import org.littletonrobotics.frc2025.subsystems.sensors.CoralSensorIO;
import org.littletonrobotics.frc2025.subsystems.sensors.CoralSensorIOLaserCan;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.*;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.Elevator;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.ElevatorIO;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.ElevatorIOSim;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.ElevatorIOTalonFX;
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
  private Intake intake;
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
  private final Trigger dimLeds = overrides.operatorSwitch(0);
  private final Trigger disableReefAutoAlign = overrides.operatorSwitch(1);
  private final Trigger disableIntakeAutoAlign = overrides.operatorSwitch(2);
  private final Trigger disableAlgaeScoreAutoAlign = overrides.operatorSwitch(3);
  private final Trigger forceSimpleCoralStrategy = overrides.operatorSwitch(4);

  private final Trigger aprilTagsReef = overrides.multiDirectionSwitchLeft();
  private final Trigger aprilTagNone = overrides.multiDirectionSwitchRight();
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.kInfo);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", AlertType.kInfo);
  private final Alert deadInTheWaterAlert =
      new Alert("Please select an auto routine!!! 😳", AlertType.kWarning);
  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1", 30.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2", 15.0);

  private Command deadInTheWater = Commands.none();
  private boolean coastOverride = false;

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
                  new VisionIONorthstar(this::getSelectedAprilTagLayout, 1),
                  new VisionIONorthstar(this::getSelectedAprilTagLayout, 2));
          elevator = new Elevator(new ElevatorIOTalonFX());
          dispenser =
              new Dispenser(
                  new PivotIOTalonFX(),
                  new RollerSystemIOTalonFX(6, "", 40, false, true, 3.0),
                  new RollerSystemIOTalonFX(7, "", 40, false, true, (30.0 / 12.0) * (48.0 / 18.0)),
                  new CoralSensorIOLaserCan(55));
          intake =
              new Intake(
                  new SlamIOTalonFX(),
                  new RollerSystemIOTalonFX(4, "", 60, true, false, 1.0),
                  new RollerSystemIOTalonFX(8, "", 40, true, false, 1.0),
                  new CoralSensorIOLaserCan(56));
          climber =
              new Climber(
                  new ClimberIOTalonFX(), new RollerSystemIOTalonFX(2, "", 60, true, false, 3.4));
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
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 3.0, 0.01),
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 2.9, 0.01),
                  new CoralSensorIO() {});
          climber =
              new Climber(
                  new ClimberIOSim(), new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 3.4, 0.05));
          intake =
              new Intake(
                  new SlamIOSim(),
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 4.0, 0.005),
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 2.0, 0.01),
                  new CoralSensorIO() {});
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
                new Vision(
                    this::getSelectedAprilTagLayout,
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {});
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
    if (intake == null) {
      intake =
          new Intake(
              new SlamIO() {},
              new RollerSystemIO() {},
              new RollerSystemIO() {},
              new CoralSensorIO() {});
    }
    if (climber == null) {
      climber = new Climber(new ClimberIO() {}, new RollerSystemIO() {});
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
    LoggedDashboardChooser<Boolean> upInThePush = new LoggedDashboardChooser<>("Up In The Push?");
    upInThePush.addDefaultOption("No", false);
    upInThePush.addOption("Yes", true);
    MirrorUtil.setMirror(mirror::get);

    // Set up characterization routines
    var autoBuilder = new AutoBuilder(drive, superstructure, intake, upInThePush::get);
    autoChooser.addDefaultOption("Dead In The Water Auto", deadInTheWater);
    autoChooser.addOption("Up In The Air Auto", autoBuilder.upInTheAirAuto());
    autoChooser.addOption("Up In The Simplicity Auto", autoBuilder.upInTheSimplicityAuto());
    autoChooser.addOption("Up In The Inspirational Auto", autoBuilder.upInTheInspirationalAuto());
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
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
    superstructure.setDisableOverride(superstructureDisable);
    elevator.setOverrides(() -> coastOverride, superstructureDisable);
    dispenser.setOverrides(() -> coastOverride, superstructureDisable);
    intake.setCoastOverride(() -> coastOverride);
    climber.setCoastOverride(() -> coastOverride);
    objectiveTracker.setOverrides(forceSimpleCoralStrategy);
    Leds.getInstance().setShouldDimSupplier(dimLeds);

    // Set up communication for superstructure and intake
    superstructure.setIntakeInterface(intake::setSuperstructureReady, intake::setHasCoral);
    intake.setCoralIndexedConsumer(superstructure::setCoralIndexed);

    // Configure the button bindings
    configureButtonBindings();

    // Warm up objective tracker
    for (int i = 0; i < 50; i++) {
      objectiveTracker.getAlgaeObjective();
      objectiveTracker.getFirstLevel();
      objectiveTracker.getSecondLevel();
      for (var level : ReefLevel.values()) {
        objectiveTracker.getCoralObjective(level, true);
        objectiveTracker.getSuperCoralObjective(new AlgaeObjective(0), level);
      }
    }
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
    objectiveTracker.setDriverInput(driverX, driverY, driverOmega, robotRelative);

    // Joystick drive command (driver and operator)
    Supplier<Command> joystickDriveCommandFactory =
        () -> DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega, robotRelative);
    drive.setDefaultCommand(joystickDriveCommandFactory.get());

    // ***** DRIVER CONTROLLER *****

    // Bind coral score function
    final Trigger firstPriorityTrigger = driver.rightTrigger();
    final Trigger secondPriorityTrigger = driver.rightBumper();
    final Trigger coralIntakingTrigger = driver.leftTrigger();
    BiConsumer<Trigger, Boolean> bindAutoScore =
        (trigger, firstPriority) -> {
          Container<ReefLevel> lockedReefLevel = new Container<>();
          Supplier<Optional<ReefLevel>> levelSupplier =
              firstPriority ? objectiveTracker::getFirstLevel : objectiveTracker::getSecondLevel;
          // Normal auto score
          Container<Boolean> autoScoreRunning = new Container<>(false);
          Trigger autoScoreAvailable =
              new Trigger(
                  () ->
                      levelSupplier
                          .get()
                          .filter(
                              reefLevel ->
                                  objectiveTracker
                                      .getCoralObjective(reefLevel, firstPriority)
                                      .isPresent())
                          .isPresent());
          trigger
              .and(trigger.doublePress().negate())
              .and(autoScoreAvailable)
              .onTrue(Commands.runOnce(() -> lockedReefLevel.value = levelSupplier.get().get()))
              .whileTrue(
                  AutoScoreCommands.autoScore(
                          drive,
                          superstructure,
                          () -> lockedReefLevel.value,
                          () ->
                              objectiveTracker.getCoralObjective(
                                  lockedReefLevel.value, firstPriority),
                          driverX,
                          driverY,
                          driverOmega,
                          joystickDriveCommandFactory.get(),
                          this::controllerRumbleCommand,
                          trigger,
                          disableReefAutoAlign,
                          driver.b().doublePress())
                      .deadlineFor(
                          Commands.startEnd(
                              () -> autoScoreRunning.value = true,
                              () -> autoScoreRunning.value = false))
                      .withName("Auto Score Priority #" + (firstPriority ? 1 : 2)));
          trigger
              .and(trigger.doublePress().negate())
              .and(autoScoreAvailable.negate())
              .and(() -> !autoScoreRunning.value)
              .onTrue(
                  Commands.sequence(
                      controllerRumbleCommand().withTimeout(0.1),
                      Commands.waitSeconds(0.1),
                      controllerRumbleCommand().withTimeout(0.1)));
          // Super auto score
          Container<Boolean> superAutoScoreRunning = new Container<>(false);
          Container<Boolean> hasAlgae = new Container<>(false);
          Container<FieldConstants.AlgaeObjective> superScoreAlgae = new Container<>(null);
          Trigger superScoreAvailable =
              new Trigger(
                  () ->
                      levelSupplier
                          .get()
                          .filter(reefLevel -> objectiveTracker.getAlgaeObjective().isPresent())
                          .isPresent());
          trigger
              .doublePress()
              .and(superScoreAvailable)
              .onTrue(
                  Commands.runOnce(
                      () -> {
                        lockedReefLevel.value = levelSupplier.get().get();
                        hasAlgae.value = superstructure.hasAlgae();
                        superScoreAlgae.value = objectiveTracker.getAlgaeObjective().get();
                      }))
              .and(() -> !hasAlgae.value)
              .whileTrue(
                  AutoScoreCommands.superAutoScore(
                          drive,
                          superstructure,
                          () -> lockedReefLevel.value,
                          () ->
                              objectiveTracker.getSuperCoralObjective(
                                  superScoreAlgae.value, lockedReefLevel.value),
                          driverX,
                          driverY,
                          driverOmega,
                          joystickDriveCommandFactory,
                          this::controllerRumbleCommand,
                          trigger.doublePress(),
                          disableReefAutoAlign,
                          driver.b().doublePress())
                      .deadlineFor(
                          Commands.startEnd(
                              () -> superAutoScoreRunning.value = true,
                              () -> superAutoScoreRunning.value = false))
                      .withName("Super Auto Score Priority #" + (firstPriority ? 1 : 2)));
          trigger
              .doublePress()
              .and(superScoreAvailable.negate())
              .and(() -> !superAutoScoreRunning.value)
              .onTrue(
                  Commands.sequence(
                      controllerRumbleCommand().withTimeout(0.1),
                      Commands.waitSeconds(0.1),
                      controllerRumbleCommand().withTimeout(0.1)));
        };
    // Score coral #1
    bindAutoScore.accept(firstPriorityTrigger, true);
    // Score coral #2
    bindAutoScore.accept(secondPriorityTrigger, false);

    // Intake commands
    Trigger scoringTriggers = firstPriorityTrigger.or(secondPriorityTrigger);
    Trigger hasCoralTrigger =
        new Trigger(() -> intake.isCoralIndexed() || superstructure.hasCoral());
    Container<Boolean> hasCoral = new Container<>(false);
    coralIntakingTrigger.onTrue(
        Commands.runOnce(() -> hasCoral.value = hasCoralTrigger.getAsBoolean()));
    coralIntakingTrigger
        .and(coralIntakingTrigger.doublePress().negate())
        .and(() -> !hasCoral.value)
        .and(scoringTriggers.negate())
        .whileTrue(
            IntakeCommands.simpleLittleAutomation(
                    drive,
                    intake,
                    hasCoralTrigger,
                    driverX,
                    driverY,
                    driverOmega,
                    joystickDriveCommandFactory.get(),
                    robotRelative,
                    disableIntakeAutoAlign)
                .alongWith(superstructure.runGoal(SuperstructureState.CORAL_INTAKE))
                .withName("Coral Intake (SLA)"))
        .onFalse(intake.retract());
    coralIntakingTrigger
        .doublePress()
        .and(() -> !hasCoral.value)
        .and(scoringTriggers.negate())
        .whileTrue(
            IntakeCommands.teleopAutoIntake(
                    drive,
                    intake,
                    hasCoralTrigger,
                    driverX,
                    driverY,
                    driverOmega,
                    joystickDriveCommandFactory.get(),
                    robotRelative,
                    disableIntakeAutoAlign)
                .alongWith(superstructure.runGoal(SuperstructureState.CORAL_INTAKE))
                .withName("Coral Auto Intake"))
        .onFalse(intake.retract());
    coralIntakingTrigger.whileTrue(
        Commands.waitUntil(hasCoralTrigger).andThen(controllerRumbleCommand().withTimeout(0.3)));
    coralIntakingTrigger
        .and(hasCoralTrigger)
        .whileTrueContinuous(
            Commands.startEnd(
                () -> Leds.getInstance().coralGrabbed = true,
                () -> Leds.getInstance().coralGrabbed = false));
    // L1 with intake
    coralIntakingTrigger
        .doublePress()
        .and(() -> hasCoral.value)
        .onTrue(intake.l1Sequence(driver.b(), hasCoralTrigger))
        .onTrue(
            superstructure
                .runGoal(SuperstructureState.REVERSE_CORAL_EJECT)
                .until(() -> !superstructure.hasCoral()));

    // Unslam
    driver.rightStick().onTrue(intake.retract().withName("Undeploy Slam"));

    // Climbing controls
    driver
        .y()
        .doublePress()
        .onTrue(
            intake
                .prepareToClimb()
                .alongWith(climber.readyClimb().withName("Ready Climber & Intake")))
        .onTrue(controllerRumbleCommand().withTimeout(0.25));

    // Algae reef intake & score
    Trigger onOpposingSide =
        new Trigger(
            () ->
                AllianceFlipUtil.applyX(RobotState.getInstance().getEstimatedPose().getX())
                    > FieldConstants.fieldLength / 2);
    Trigger shouldProcess =
        new Trigger(
            () ->
                AllianceFlipUtil.apply(
                                RobotState.getInstance()
                                    .getEstimatedPose()
                                    .exp(
                                        RobotState.getInstance()
                                            .getRobotVelocity()
                                            .toTwist2d(
                                                AlgaeScoreCommands.getLookaheadSecs().get())))
                            .getY()
                        < FieldConstants.fieldWidth / 2 - DriveConstants.robotWidth
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

    // Algae ground intake
    driver
        .x()
        .toggleOnTrue(
            IntakeCommands.algaeIntakeWithAiming(
                    drive,
                    superstructure,
                    driverX,
                    driverY,
                    driverOmega,
                    robotRelative,
                    disableIntakeAutoAlign)
                .withName("Algae Ground Intake"));

    // Algae pre-processor
    driver
        .leftBumper()
        .and(shouldProcess)
        .and(() -> hasAlgae.value)
        .and(driver.a().negate())
        .or(AlgaeScoreCommands::shouldForceProcess)
        .whileTrueContinuous(
            AlgaeScoreCommands.process(
                    drive,
                    superstructure,
                    driverX,
                    driverY,
                    driverOmega,
                    joystickDriveCommandFactory,
                    onOpposingSide,
                    driver.leftBumper(),
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
                    joystickDriveCommandFactory,
                    onOpposingSide,
                    driver.leftBumper(),
                    true,
                    disableAlgaeScoreAutoAlign)
                .withName("Algae Processing"));

    // Algae net score
    driver
        .leftBumper()
        .and(shouldProcess.negate())
        .and(() -> hasAlgae.value)
        .whileTrue(
            AlgaeScoreCommands.netScore(
                    drive,
                    superstructure,
                    driverX,
                    driverY,
                    joystickDriveCommandFactory.get(),
                    driver.a(),
                    controllerRumbleCommand(),
                    disableAlgaeScoreAutoAlign)
                .withName("Algae Net Score"));

    // Algae eject
    driver
        .a()
        .and(driver.leftBumper().negate())
        .whileTrue(superstructure.runGoal(SuperstructureState.TOSS).withName("Algae Toss"));

    // Run intake reverse
    driver.leftStick().whileTrue(intake.outtake());

    // Coral eject
    driver
        .b()
        .and(firstPriorityTrigger.negate())
        .and(secondPriorityTrigger.negate())
        .whileTrue(superstructure.runTunnelEject().withName("Coral Eject"));

    // Nudge climber up and down
    driver
        .povLeft()
        .and(() -> climber.getClimbState() == ClimbState.PULL)
        .onTrue(Commands.runOnce(() -> climber.adjustClimbOffset(-5)));
    driver
        .povRight()
        .and(() -> climber.getClimbState() == ClimbState.PULL)
        .onTrue(Commands.runOnce(() -> climber.adjustClimbOffset(5)));

    // Raise elevator
    driver
        .povUp()
        .toggleOnTrue(
            superstructure.runGoal(SuperstructureState.L3_CORAL).withName("Force Raise Elevator"));

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
    operator.rightBumper().whileTrue(intake.intake());

    // Run intake reverse
    operator.rightStick().whileTrue(intake.outtake());

    // Home elevator & intake
    operator
        .leftBumper()
        .onTrue(
            superstructure
                .runHomingSequence()
                .alongWith(intake.runHomingSequence())
                .withName("Operator Home Elevator & Intake"));

    // Force net
    operator
        .povLeft()
        .whileTrue(
            superstructure.runGoal(SuperstructureState.THROW).withName("Operator Force Net"));

    // Force processor
    operator
        .povRight()
        .whileTrue(
            superstructure
                .runGoal(SuperstructureState.PROCESS)
                .withName("Operator Force Processor"));

    // Algae eject
    operator
        .rightTrigger()
        .and(operator.a().negate())
        .and(operator.b().negate())
        .and(operator.x().negate())
        .and(operator.y().negate())
        .whileTrue(superstructure.forceEjectDispenser().withName("Operator Force Eject"));

    // Operator commands for superstructure
    BiConsumer<Trigger, ReefLevel> bindOperatorCoralScore =
        (faceButton, height) -> {
          faceButton.whileTrueContinuous(
              superstructure
                  .runGoal(
                      () ->
                          Superstructure.getScoringState(height, superstructure.hasAlgae(), false))
                  .withName("Operator Score on " + height));
          faceButton
              .and(operator.rightTrigger())
              .whileTrueContinuous(
                  superstructure
                      .runGoal(
                          () ->
                              Superstructure.getScoringState(
                                  height, superstructure.hasAlgae(), true))
                      .withName("Operator Score & Eject On " + height));
        };
    bindOperatorCoralScore.accept(operator.a(), ReefLevel.L1);
    bindOperatorCoralScore.accept(operator.x(), ReefLevel.L2);
    bindOperatorCoralScore.accept(operator.b(), ReefLevel.L3);
    bindOperatorCoralScore.accept(operator.y(), ReefLevel.L4);

    // ***** MISCELlANEOUS *****

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
                        coastOverride = true;
                        leds.superstructureCoast = true;
                      }
                    })
                .withName("Superstructure Coast")
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(
                    () -> {
                      coastOverride = false;
                      leds.superstructureCoast = false;
                    })
                .withName("Superstructure Uncoast")
                .ignoringDisable(true));
    RobotModeTriggers.disabled()
        .onFalse(
            Commands.runOnce(
                    () -> {
                      coastOverride = false;
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

    // Coral grabbed alert
    new Trigger(() -> superstructure.hasCoral() || superstructure.isCoralIndexed())
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.25)
                .beforeStarting(() -> leds.coralGrabbed = true)
                .finallyDo(() -> leds.coralGrabbed = false));

    // Algae grabbed alert
    new Trigger(superstructure::hasAlgae)
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(.25)
                .beforeStarting(() -> leds.algaeGrabbed = true)
                .finallyDo(() -> leds.algaeGrabbed = false));
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

    // Auto alert
    deadInTheWaterAlert.set(
        DriverStation.isAutonomous()
            && !DriverStation.isEnabled()
            && autoChooser.get() == deadInTheWater);
  }

  /** Returns the current AprilTag layout type. */
  public AprilTagLayoutType getSelectedAprilTagLayout() {
    if (aprilTagsReef.getAsBoolean()) {
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
        return FieldConstants.AprilTagLayoutType.BLUE_REEF;
      } else {
        return FieldConstants.AprilTagLayoutType.RED_REEF;
      }
    } else if (aprilTagNone.getAsBoolean()) {
      return FieldConstants.AprilTagLayoutType.NONE;
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
