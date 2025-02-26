// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2025.Constants.Mode;
import org.littletonrobotics.frc2025.FieldConstants.AprilTagLayoutType;
import org.littletonrobotics.frc2025.FieldConstants.ReefLevel;
import org.littletonrobotics.frc2025.commands.*;
import org.littletonrobotics.frc2025.subsystems.climber.Climber;
import org.littletonrobotics.frc2025.subsystems.climber.ClimberIO;
import org.littletonrobotics.frc2025.subsystems.climber.ClimberIOSim;
import org.littletonrobotics.frc2025.subsystems.climber.ClimberIOTalonFX;
import org.littletonrobotics.frc2025.subsystems.drive.*;
import org.littletonrobotics.frc2025.subsystems.leds.Leds;
import org.littletonrobotics.frc2025.subsystems.objectivetracker.ObjectiveTracker;
import org.littletonrobotics.frc2025.subsystems.objectivetracker.ReefControlsIO;
import org.littletonrobotics.frc2025.subsystems.objectivetracker.ReefControlsIOServer;
import org.littletonrobotics.frc2025.subsystems.rollers.*;
import org.littletonrobotics.frc2025.subsystems.superstructure.Superstructure;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureConstants;
import org.littletonrobotics.frc2025.subsystems.superstructure.SuperstructureState;
import org.littletonrobotics.frc2025.subsystems.superstructure.dispenser.*;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.Elevator;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.ElevatorIO;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.ElevatorIOSim;
import org.littletonrobotics.frc2025.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import org.littletonrobotics.frc2025.subsystems.vision.Vision;
import org.littletonrobotics.frc2025.subsystems.vision.VisionIO;
import org.littletonrobotics.frc2025.subsystems.vision.VisionIONorthstar;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.Container;
import org.littletonrobotics.frc2025.util.DoublePressTracker;
import org.littletonrobotics.frc2025.util.OverrideSwitches;
import org.littletonrobotics.frc2025.util.TriggerUtil;
import org.littletonrobotics.frc2025.util.trajectory.HolonomicTrajectory;
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
  private final Trigger disableAutoCoralStationIntake = overrides.operatorSwitch(0);
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
  private boolean climberDeployed = false;

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
          // vision =
          //     new Vision(
          //         this::getSelectedAprilTagLayout,
          //         new VisionIONorthstar(this::getSelectedAprilTagLayout, 0),
          //         new VisionIONorthstar(this::getSelectedAprilTagLayout, 1),
          //         new VisionIONorthstar(this::getSelectedAprilTagLayout, 2),
          //         new VisionIONorthstar(this::getSelectedAprilTagLayout, 3));
          elevator = new Elevator(new ElevatorIOTalonFX());
          dispenser =
              new Dispenser(
                  new DispenserIOTalonFX(),
                  new RollerSystemIOTalonFX(6, "", 40, false, false, 3.0),
                  new RollerSystemIOTalonFX(7, "", 40, false, false, 2.0));
          // chariot =
          //     new Chariot(
          //         new ChariotIOTalonFX(), new RollerSystemIOTalonFX(12, "*", 0, false, false,
          // 1.0));
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
                  new DispenserIO() {}, new RollerSystemIOSpark(5, true), new RollerSystemIO() {});
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
                  new DispenserIOSim(),
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.2),
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.2));
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
                new Vision(
                    this::getSelectedAprilTagLayout,
                    new VisionIO() {},
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
          new Dispenser(new DispenserIO() {}, new RollerSystemIO() {}, new RollerSystemIO() {});
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

    // Set up Characterization routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    HolonomicTrajectory testTrajectory = new HolonomicTrajectory("BLOB");
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
          Supplier<Optional<ReefLevel>> levelSupplier =
              firstPriority ? objectiveTracker::getFirstLevel : objectiveTracker::getSecondLevel;
          trigger
              .and(
                  () ->
                      levelSupplier
                          .get()
                          .filter(
                              reefLevel ->
                                  objectiveTracker.getCoralObjective(reefLevel).isPresent())
                          .isPresent())
              .onTrue(Commands.runOnce(() -> lockedReefLevel.value = levelSupplier.get().get()))
              .whileTrue(
                  AutoScore.getAutoScoreCommand(
                          drive,
                          superstructure,
                          funnel,
                          objectiveTracker::requestScored,
                          () -> lockedReefLevel.value,
                          () -> objectiveTracker.getCoralObjective(lockedReefLevel.value),
                          driverX,
                          driverY,
                          driverOmega,
                          joystickDriveCommandFactory.get(),
                          disableReefAutoAlign)
                      .withName("Auto Score Priority #" + (firstPriority ? 1 : 2)));
        };
    // Score coral #1
    bindAutoScore.accept(driver.rightTrigger(), true);
    // Score coral #2
    bindAutoScore.accept(driver.rightBumper(), false);

    // Climbing controls
    driver
        .y()
        .and(() -> !climberDeployed)
        .doublePress()
        .onTrue(climber.deploy().alongWith(Commands.runOnce(() -> climberDeployed = true)));
    driver.y().and(() -> climberDeployed).doublePress().whileTrue(climber.climb());
    RobotModeTriggers.disabled()
        .onTrue(Commands.runOnce(() -> climberDeployed = false).ignoringDisable(true));

    // Coral intake
    driver
        .leftTrigger()
        .whileTrue(
            Commands.either(
                    joystickDriveCommandFactory.get(),
                    new DriveToStation(drive, driverX, driverY, driverOmega, true),
                    disableCoralStationAutoAlign)
                .alongWith(IntakeCommands.intake(superstructure, funnel))
                .withName("Coral Station Intake"));

    // Algae reef intake & score
    Trigger shouldProcess =
        new Trigger(
            () ->
                AllianceFlipUtil.applyY(RobotState.getInstance().getEstimatedPose().getY())
                    < FieldConstants.fieldWidth / 2);
    Container<Boolean> hasAlgae = new Container<>(false);
    driver.leftBumper().onTrue(Commands.runOnce(() -> hasAlgae.value = superstructure.hasAlgae()));

    // Algae reef intake
    driver
        .leftBumper()
        .and(() -> !hasAlgae.value)
        .whileTrue(
            AutoScore.getReefIntakeCommand(
                    drive,
                    superstructure,
                    objectiveTracker::requestAlgaeIntaked,
                    objectiveTracker::getAlgaeObjective,
                    driverX,
                    driverY,
                    driverOmega,
                    joystickDriveCommandFactory.get(),
                    disableReefAutoAlign)
                .onlyIf(() -> objectiveTracker.getAlgaeObjective().isPresent())
                .withName("Algae Reef Intake"));

    // Operator command for algae intake
    Function<Boolean, Command> algaeProcessCommand =
        eject ->
            Commands.either(
                    joystickDriveCommandFactory.get(),
                    new DriveToPose(
                        drive,
                        () ->
                            AutoScore.getDriveTarget(
                                RobotState.getInstance().getEstimatedPose(),
                                AllianceFlipUtil.apply(
                                    FieldConstants.Processor.centerFace.transformBy(
                                        new Transform2d(
                                            new Translation2d(
                                                DriveConstants.robotWidth / 2.0
                                                    + Units.inchesToMeters(3.0),
                                                0),
                                            Rotation2d.kPi)))),
                        RobotState.getInstance()::getEstimatedPose,
                        () ->
                            DriveCommands.getLinearVelocityFromJoysticks(
                                    driverX.getAsDouble(), driverY.getAsDouble())
                                .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
                        () -> DriveCommands.getOmegaFromJoysticks(driverOmega.getAsDouble())),
                    disableAlgaeScoreAutoAlign)
                .alongWith(
                    eject
                        ? superstructure.runGoal(SuperstructureState.PROCESSED)
                        : superstructure.runGoal(SuperstructureState.POST_PRE_PROCESSOR));

    // Algae pre-processor
    driver
        .leftBumper()
        .and(shouldProcess)
        .and(() -> hasAlgae.value)
        .and(driver.a().negate())
        .whileTrueContinuous(algaeProcessCommand.apply(false).withName("Algae Pre-Processor"));

    // Algae process
    driver
        .leftBumper()
        .and(shouldProcess)
        .and(() -> hasAlgae.value)
        .and(driver.a())
        .whileTrueContinuous(algaeProcessCommand.apply(true).withName("Algae Processing"));

    Function<Boolean, Command> algaeNetCommand =
        eject -> {
          var autoAlignCommand =
              new DriveToPose(
                  drive,
                  () ->
                      new Pose2d(
                          AllianceFlipUtil.applyX(
                              FieldConstants.fieldLength / 2.0
                                  - FieldConstants.Barge.netWidth / 2.0
                                  - FieldConstants.algaeDiameter
                                  - SuperstructureConstants.pivotToTunnelFront
                                      * Math.cos(20.0 / 180.0 * Math.PI)
                                  - SuperstructureConstants.elevatorMaxTravel
                                      * SuperstructureConstants.elevatorAngle.getCos()
                                  - SuperstructureConstants.dispenserOrigin2d.getX()
                                  - Units.inchesToMeters(5.0)),
                          RobotState.getInstance().getEstimatedPose().getY(),
                          Rotation2d.kZero),
                  RobotState.getInstance()::getEstimatedPose,
                  () ->
                      DriveCommands.getLinearVelocityFromJoysticks(0, driverY.getAsDouble())
                          .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0),
                  () -> 0);

          return Commands.either(
                  joystickDriveCommandFactory.get(), autoAlignCommand, disableAlgaeScoreAutoAlign)
              .alongWith(
                  Commands.waitUntil(
                          () ->
                              disableAlgaeScoreAutoAlign.getAsBoolean()
                                  || (autoAlignCommand.isRunning()
                                      && autoAlignCommand.withinTolerance(
                                          Units.inchesToMeters(20.0),
                                          Rotation2d.fromDegrees(10.0))))
                      .andThen(
                          eject
                              ? superstructure.runGoal(SuperstructureState.THROWN)
                              : superstructure.runGoal(SuperstructureState.PRE_THROWN)));
        };

    // Algae pre-net
    driver
        .leftBumper()
        .and(shouldProcess.negate())
        .and(() -> hasAlgae.value)
        .and(driver.a().negate())
        .whileTrueContinuous(algaeNetCommand.apply(false).withName("Algae Pre-Net"));

    // Algae net score
    driver
        .leftBumper()
        .and(shouldProcess.negate())
        .and(() -> hasAlgae.value)
        .and(driver.a())
        .whileTrueContinuous(algaeNetCommand.apply(true).withName("Algae Net Score"));

    // Algae eject
    driver
        .a()
        .and(driver.leftBumper().negate())
        .whileTrue(superstructure.runGoal(SuperstructureState.TOSS).withName("Algae Toss"));

    // Strobe LEDs at human player
    driver
        .y()
        .whileTrue(
            Commands.startEnd(
                () -> leds.hpAttentionAlert = true, () -> leds.hpAttentionAlert = false));

    // Coral eject
    driver.b().whileTrue(superstructure.runGoal(SuperstructureState.L1_CORAL_EJECT));

    // Force net
    driver.povLeft().whileTrue(superstructure.runGoal(SuperstructureState.THROWN));

    // Force processor
    driver.povRight().whileTrue(superstructure.runGoal(SuperstructureState.PROCESSED));

    // ***** OPERATOR CONTROLLER *****

    // Algae stow intake
    operator
        .leftTrigger()
        .whileTrue(
            superstructure
                .runGoal(SuperstructureState.ALGAE_STOW_INTAKE)
                .withName("Algae Stow Intake"));

    // Coral intake
    operator.rightBumper().whileTrue(IntakeCommands.intake(superstructure, funnel));

    // Home elevator
    operator.leftBumper().onTrue(superstructure.runHomingSequence());

    // Force net
    operator.povLeft().whileTrue(superstructure.runGoal(SuperstructureState.THROWN));

    // Force processor
    operator.povRight().whileTrue(superstructure.runGoal(SuperstructureState.PROCESSED));

    // Algae eject
    operator
        .rightTrigger()
        .and(operator.a().negate())
        .and(operator.b().negate())
        .and(operator.x().negate())
        .and(operator.y().negate())
        .whileTrue(superstructure.runGoal(SuperstructureState.TOSS));

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

    // Auto intake coral
    funnel.setDefaultCommand(
        funnel.runRoller(
            () -> superstructure.isRequestFunnelIntake() ? IntakeCommands.funnelVolts.get() : 0.0));

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
                                    AllianceFlipUtil.apply(new Rotation2d()))))
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
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(
                    () -> {
                      superstructureCoastOverride = false;
                      leds.superstructureCoast = false;
                    })
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
                .finallyDo(() -> leds.endgameAlert = false));
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
                .finallyDo(() -> leds.endgameAlert = false)); // Rumble three times
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
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operator.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operator.getHID().getPort()));
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
