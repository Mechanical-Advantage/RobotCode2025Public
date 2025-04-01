// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "RobotContainer.h"

#include <functional>
#include <optional>
#include <vector>

#include <frc/Alert.h>
#include <frc/DriverStation.h>
#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc2/command/button/Trigger.h>

#include "Constants.h"
#include "FieldConstants.h"
#include "commands/AlgaeScoreCommands.h"
#include "commands/AutoScoreCommands.h"
#include "commands/DriveCommands.h"
#include "commands/DriveTrajectory.h"
#include "commands/IntakeCommands.h"
#include "junction/networktables/LoggedDashboardChooser.h"
#include "junction/networktables/LoggedNetworkNumber.h"
#include "subsystems/climber/Climber.h"
#include "subsystems/climber/ClimberIO.h"
#include "subsystems/climber/ClimberIOSim.h"
#include "subsystems/climber/ClimberIOTalonFX.h"
#include "subsystems/drive/Drive.h"
#include "subsystems/drive/GyroIO.h"
#include "subsystems/drive/GyroIOPigeon2.h"
#include "subsystems/drive/GyroIORedux.h"
#include "subsystems/drive/ModuleIO.h"
#include "subsystems/drive/ModuleIOComp.h"
#include "subsystems/drive/ModuleIODev.h"
#include "subsystems/drive/ModuleIOSim.h"
#include "subsystems/drive/trajectory/HolonomicTrajectory.h"
#include "subsystems/leds/Leds.h"
#include "subsystems/objectivetracker/ObjectiveTracker.h"
#include "subsystems/objectivetracker/ReefControlsIO.h"
#include "subsystems/objectivetracker/ReefControlsIOServer.h"
#include "subsystems/rollers/RollerSystem.h"
#include "subsystems/rollers/RollerSystemIO.h"
#include "subsystems/rollers/RollerSystemIOSim.h"
#include "subsystems/rollers/RollerSystemIOSpark.h"
#include "subsystems/rollers/RollerSystemIOTalonFX.h"
#include "subsystems/superstructure/Superstructure.h"
#include "subsystems/superstructure/dispenser/Dispenser.h"
#include "subsystems/superstructure/dispenser/PivotIO.h"
#include "subsystems/superstructure/dispenser/PivotIOSim.h"
#include "subsystems/superstructure/dispenser/PivotIOTalonFX.h"
#include "subsystems/superstructure/elevator/Elevator.h"
#include "subsystems/superstructure/elevator/ElevatorIO.h"
#include "subsystems/superstructure/elevator/ElevatorIOSim.h"
#include "subsystems/superstructure/elevator/ElevatorIOTalonFX.h"
#include "subsystems/superstructure/sensors/CoralSensorIO.h"
#include "subsystems/superstructure/sensors/CoralSensorIOLaserCan.h"
#include "subsystems/vision/Vision.h"
#include "subsystems/vision/VisionIO.h"
#include "subsystems/vision/VisionIONorthstar.h"
#include "util/Container.h"
#include "util/DoublePressTracker.h"
#include "util/MirrorUtil.h"
#include "util/OverrideSwitches.h"
#include "util/RobotState.h"
#include "util/TriggerUtil.h"

RobotContainer::RobotContainer() {
  Elevator *elevator = nullptr;
  Dispenser *dispenser = nullptr;

  if (Constants::GetMode() != Constants::Mode::kReplay) {
    switch (Constants::GetRobot()) {
    case Constants::Robot::kCompbot: {
      drive = new Drive(new GyroIOPigeon2(),
                        new ModuleIOComp(DriveConstants::moduleConfigsComp[0]),
                        new ModuleIOComp(DriveConstants::moduleConfigsComp[1]),
                        new ModuleIOComp(DriveConstants::moduleConfigsComp[2]),
                        new ModuleIOComp(DriveConstants::moduleConfigsComp[3]));
      vision = new Vision(
          std::bind(&RobotContainer::getSelectedAprilTagLayout, this),
          new VisionIONorthstar(
              std::bind(&RobotContainer::getSelectedAprilTagLayout, this), 0),
          new VisionIONorthstar(
              std::bind(&RobotContainer::getSelectedAprilTagLayout, this), 1));
      elevator = new Elevator(new ElevatorIOTalonFX());
      dispenser = new Dispenser(
          new PivotIOTalonFX(),
          new RollerSystemIOTalonFX(6, "", 40, true, false, 3.0),
          new RollerSystemIOTalonFX(7, "", 40, true, false,
                                    (30.0 / 12.0) * (48.0 / 18.0)),
          new CoralSensorIOLaserCan());
      funnel = new RollerSystem(
          "Funnel", new RollerSystemIOTalonFX(2, "", 30, true, false, 1.0));
      climber = new Climber(new ClimberIOTalonFX());
      break;
    }
    case Constants::Robot::kDevbot: {
      drive = new Drive(new GyroIORedux(),
                        new ModuleIODev(DriveConstants::moduleConfigsDev[0]),
                        new ModuleIODev(DriveConstants::moduleConfigsDev[1]),
                        new ModuleIODev(DriveConstants::moduleConfigsDev[2]),
                        new ModuleIODev(DriveConstants::moduleConfigsDev[3]));
      vision = new Vision(
          std::bind(&RobotContainer::getSelectedAprilTagLayout, this),
          new VisionIONorthstar(
              std::bind(&RobotContainer::getSelectedAprilTagLayout, this), 0),
          new VisionIONorthstar(
              std::bind(&RobotContainer::getSelectedAprilTagLayout, this), 1));
      elevator = new Elevator(new ElevatorIOTalonFX());
      dispenser = new Dispenser(new PivotIO(), new RollerSystemIOSpark(5, true),
                                new RollerSystemIO(), new CoralSensorIO());
      funnel = new RollerSystem("Funnel", new RollerSystemIOSpark(4, false));
      break;
    }
    case Constants::Robot::kSimbot: {
      drive = new Drive(new GyroIO(), new ModuleIOSim(), new ModuleIOSim(),
                        new ModuleIOSim(), new ModuleIOSim());
      elevator = new Elevator(new ElevatorIOSim());
      dispenser = new Dispenser(
          new PivotIOSim(),
          new RollerSystemIOSim(frc::DCMotor::KrakenX60Foc(1), 1.0, 0.2),
          new RollerSystemIOSim(frc::DCMotor::KrakenX60Foc(1), 1.0, 0.2),
          new CoralSensorIO());
      climber = new Climber(new ClimberIOSim());
      funnel = new RollerSystem(
          "Funnel",
          new RollerSystemIOSim(frc::DCMotor::KrakenX60Foc(1), 1.0, 0.02));
      break;
    }
    }
  }

  // No-op implementations for replay
  if (drive == nullptr) {
    drive = new Drive(new GyroIO(), new ModuleIO(), new ModuleIO(),
                      new ModuleIO(), new ModuleIO());
  }
  if (vision == nullptr) {
    switch (Constants::GetRobot()) {
    case Constants::Robot::kCompbot:
      vision = new Vision(
          std::bind(&RobotContainer::getSelectedAprilTagLayout, this),
          new VisionIO(), new VisionIO());
      break;
    case Constants::Robot::kDevbot:
      vision = new Vision(
          std::bind(&RobotContainer::getSelectedAprilTagLayout, this),
          new VisionIO());
      break;
    default:
      vision = new Vision(
          std::bind(&RobotContainer::getSelectedAprilTagLayout, this));
      break;
    }
  }
  if (elevator == nullptr) {
    elevator = new Elevator(new ElevatorIO());
  }
  if (dispenser == nullptr) {
    dispenser = new Dispenser(new PivotIO(), new RollerSystemIO(),
                              new RollerSystemIO(), new CoralSensorIO());
  }
  if (funnel == nullptr) {
    funnel = new RollerSystem("Funnel", new RollerSystemIO());
  }
  if (climber == nullptr) {
    climber = new Climber(new ClimberIO());
  }
  objectiveTracker =
      new ObjectiveTracker(Constants::GetMode() == Constants::Mode::kReplay
                               ? new ReefControlsIO()
                               : new ReefControlsIOServer());
  superstructure = new Superstructure(elevator, dispenser);

  // Set up auto routines
  autoChooser =
      LoggedDashboardChooser<frc2::command::Command *>("Auto Choices");
  LoggedDashboardChooser<bool> mirror =
      LoggedDashboardChooser<bool>("Processor Side?");
  mirror.AddDefaultOption("Yes", false);
  mirror.AddOption("No", true);
  MirrorUtil::SetMirror(std::bind(&LoggedDashboardChooser<bool>::Get, &mirror));

  // Set up characterization routines
  AutoBuilder autoBuilder(drive, superstructure, funnel);
  autoChooser.AddDefaultOption("Dead In The Water Auto",
                               new frc2::command::InstantCommand());
  // autoChooser.AddOption("Super Up In The Water Auto",
  // autoBuilder.SuperUpInTheWaterAuto()); autoChooser.AddOption("Up In The
  // Water Auto", autoBuilder.UpInTheWaterAuto());
  autoChooser.AddOption("Up In The Weeds Auto",
                        autoBuilder.UpInTheWeedsAuto(false));
  autoChooser.AddOption("Up In The Elimination Auto",
                        autoBuilder.UpInTheWeedsAuto(true));
  autoChooser.AddOption("Up In The Simplicity Auto",
                        autoBuilder.UpInTheSimplicityAuto());
  autoChooser.AddOption("Up In The Inspirational Auto",
                        autoBuilder.UpInTheInspirationalAuto());
  autoChooser.AddOption("Drive Wheel Radius Characterization",
                        DriveCommands::WheelRadiusCharacterization(drive));
  autoChooser.AddOption("Drive Simple FF Characterization",
                        DriveCommands::FeedforwardCharacterization(drive));
  HolonomicTrajectory testTrajectory("driveStraight");
  autoChooser.AddOption(
      "Drive Trajectory",
      new frc2::command::InstantCommand([&]() {
        RobotState::GetInstance().ResetPose(
            AllianceFlipUtil::Apply(testTrajectory.GetStartPose()));
      }).AndThen(new DriveTrajectory(drive, testTrajectory)));
  autoChooser.AddOption("Elevator Static Up",
                        superstructure->SetCharacterizationMode().AndThen(
                            elevator->UpStaticCharacterization()));
  autoChooser.AddOption("Elevator Static Down",
                        superstructure->SetCharacterizationMode().AndThen(
                            elevator->DownStaticCharacterization()));
  autoChooser.AddOption("Pivot static",
                        superstructure->SetCharacterizationMode().AndThen(
                            dispenser->StaticCharacterization()));

  // Set up overrides
  superstructure->SetOverrides(superstructureDisable,
                               disableAutoCoralStationIntake);
  elevator->SetOverrides([&]() { return superstructureCoastOverride; },
                         superstructureDisable);
  dispenser->SetOverrides([&]() { return superstructureCoastOverride; },
                          superstructureDisable,
                          disableDispenserGamePieceDetection);
  climber->SetCoastOverride([&]() { return superstructureCoastOverride; });
  objectiveTracker->SetForceReefBlocked(
      std::bind(&Superstructure::HasAlgae, superstructure));

  // Configure the button bindings
  configureButtonBindings();
}

void RobotContainer::configureButtonBindings() {
  // Drive suppliers
  std::function<double()> driverX = [&]() {
    return -driver.GetLeftY() - operatorController.GetLeftY();
  };
  std::function<double()> driverY = [&]() {
    return -driver.GetLeftX() - operatorController.GetLeftX();
  };
  std::function<double()> driverOmega = [&]() {
    return -driver.GetRightX() - operatorController.GetRightX();
  };

  // Joystick drive command (driver and operator)
  std::function<frc2::command::Command *()> joystickDriveCommandFactory =
      [&]() {
        return DriveCommands::JoystickDrive(drive, driverX, driverY,
                                            driverOmega, robotRelative);
      };
  drive->SetDefaultCommand(joystickDriveCommandFactory());

  // ***** DRIVER CONTROLLER *****

  // Auto score coral
  std::function<void(frc2::command::button::Trigger &, bool)> bindAutoScore =
      [&](frc2::command::button::Trigger &trigger, bool firstPriority) {
        Container<FieldConstants::ReefLevel> lockedReefLevel;
        Container<bool> running(false);
        std::function<std::optional<FieldConstants::ReefLevel>()>
            levelSupplier =
                firstPriority ? std::bind(&ObjectiveTracker::GetFirstLevel,
                                          objectiveTracker)
                              : std::bind(&ObjectiveTracker::GetSecondLevel,
                                          objectiveTracker);
        frc2::command::button::Trigger blocked([&]() {
          return !levelSupplier().has_value() ||
                 !objectiveTracker->GetCoralObjective(levelSupplier().value())
                      .has_value();
        });
        trigger.And(blocked.Negate())
            .OnTrue(new frc2::command::InstantCommand(
                [&]() { lockedReefLevel.value = levelSupplier().value(); }))
            .WhileTrue(AutoScoreCommands::AutoScore(
                           drive, superstructure, funnel,
                           [&]() { return lockedReefLevel.value; },
                           [&]() {
                             return objectiveTracker->GetCoralObjective(
                                 lockedReefLevel.value);
                           },
                           driverX, driverY, driverOmega,
                           joystickDriveCommandFactory(), disableReefAutoAlign,
                           driver.B())
                           .DeadlineWith(new frc2::command::InstantCommand(
                               [&]() { running.value = true; },
                               [&]() { running.value = false; })))
            .WithName("Auto Score Priority #" +
                      std::to_string(firstPriority ? 1 : 2));
        trigger.And(blocked)
            .And([&]() { return !running.value; })
            .OnTrue(new frc2::command::SequentialCommandGroup(
                controllerRumbleCommand()->WithTimeout(0.1),
                new frc2::command::WaitCommand(0.1_s),
                controllerRumbleCommand()->WithTimeout(0.1)));
      };
  // Score coral #1
  bindAutoScore(driver.RightTrigger(), true);
  // Score coral #2
  bindAutoScore(driver.RightBumper(), false);

  // Climbing controls
  Container<Climber::ClimbState> climbState(Climber::ClimbState::kStart);
  driver.Y()
      .And([&]() { return climbState.value == Climber::ClimbState::kStart; })
      .DoublePress()
      .OnTrue(climber->Deploy().AndThen([&]() {
        climbState.value = Climber::ClimbState::kDeployed;
        Leds::GetInstance().ready = true;
      }));
  driver.Y()
      .And([&]() { return climbState.value == Climber::ClimbState::kDeployed; })
      .DoublePress()
      .OnTrue(climber->Climb()
                  .AlongWith(new frc2::command::InstantCommand([&]() {
                    climbState.value = Climber::ClimbState::kClimbing;
                  }))
                  .FinallyDo([&]() { Leds::GetInstance().ready = false; }));
  driver.Y()
      .And([&]() { return climbState.value == Climber::ClimbState::kClimbing; })
      .DoublePress()
      .OnTrue(climber->Undeploy().AndThen([&]() {
        climbState.value = Climber::ClimbState::kDeployed;
        Leds::GetInstance().ready = true;
      }));
  frc2::command::button::RobotModeTriggers::Disabled().OnTrue(
      new frc2::command::InstantCommand([&]() {
        climbState.value = Climber::ClimbState::kStart;
        Leds::GetInstance().ready = false;
      }).IgnoringDisable(true));

  // Coral intake
  driver.LeftTrigger().WhileTrueContinuous(
      frc2::command::Commands::Either(
          joystickDriveCommandFactory(),
          new DriveToStation(drive, driverX, driverY, driverOmega, false),
          disableCoralStationAutoAlign)
          .DeadlineWith(new frc2::command::InstantCommand(
              [&]() { Leds::GetInstance().autoScoring = true; },
              [&]() { Leds::GetInstance().autoScoring = false; }))
          .AlongWith(IntakeCommands::Intake(superstructure, funnel),
                     new frc2::command::InstantCommand(
                         [&]() { superstructure->ResetHasCoral(); }))
          .WithName("Coral Station Intake"));

  // Algae reef intake & score
  frc2::command::button::Trigger onOpposingSide([&]() {
    return AllianceFlipUtil::ApplyX(
               RobotState::GetInstance().GetEstimatedPose().X()) >
           FieldConstants::fieldLength / 2.0;
  });
  frc2::command::button::Trigger shouldProcess([&]() {
    return AllianceFlipUtil::ApplyY(
               RobotState::GetInstance().GetEstimatedPose().Y()) <
               FieldConstants::fieldWidth / 2.0 ||
           onOpposingSide.Get();
  });
  Container<bool> hasAlgae(false);
  driver.LeftBumper().OnTrue(new frc2::command::InstantCommand(
      [&]() { hasAlgae.value = superstructure->HasAlgae(); }));

  // Algae reef intake
  driver.LeftBumper()
      .And([&]() { return !hasAlgae.value; })
      .WhileTrue(
          AutoScoreCommands::ReefIntake(
              drive, superstructure,
              std::bind(&ObjectiveTracker::GetAlgaeObjective, objectiveTracker),
              driverX, driverY, driverOmega, joystickDriveCommandFactory(),
              disableReefAutoAlign)
              .DeadlineWith(new frc2::command::InstantCommand(
                  [&]() { Leds::GetInstance().autoScoring = true; },
                  [&]() { Leds::GetInstance().autoScoring = false; }))
              .OnlyIf([&]() {
                return objectiveTracker->GetAlgaeObjective().has_value();
              })
              .WithName("Algae Reef Intake"));

  // Algae pre-processor
  driver.LeftBumper()
      .And(shouldProcess)
      .And([&]() { return hasAlgae.value; })
      .And(driver.A().Negate())
      .WhileTrueContinuous(AlgaeScoreCommands::Process(
                               drive, superstructure, driverX, driverY,
                               driverOmega, joystickDriveCommandFactory(),
                               onOpposingSide, false,
                               disableAlgaeScoreAutoAlign)
                               .WithName("Algae Pre-Processor"));

  // Algae process
  driver.LeftBumper()
      .And(shouldProcess)
      .And([&]() { return hasAlgae.value; })
      .And(driver.A())
      .WhileTrueContinuous(AlgaeScoreCommands::Process(
                               drive, superstructure, driverX, driverY,
                               driverOmega, joystickDriveCommandFactory(),
                               onOpposingSide, true, disableAlgaeScoreAutoAlign)
                               .WithName("Algae Processing"));

  // Algae pre-net
  driver.LeftBumper()
      .And(shouldProcess.Negate())
      .And([&]() { return hasAlgae.value; })
      .And(driver.A().Negate())
      .WhileTrue(AlgaeScoreCommands::NetThrowLineup(
                     drive, superstructure, driverY,
                     joystickDriveCommandFactory(), disableAlgaeScoreAutoAlign)
                     .WithName("Algae Pre-Net"))
      .OnTrue(new frc2::command::InstantCommand(
          [&]() { Leds::GetInstance().autoScoring = true; }))
      .OnFalse(new frc2::command::InstantCommand(
          [&]() { Leds::GetInstance().autoScoring = false; }))
      // Indicate ready for score
      .And([&]() {
        return superstructure->GetState() ==
               Superstructure::SuperstructureState::kPreThrown;
      })
      .WhileTrue(controllerRumbleCommand()
                     ->WithTimeout(0.1)
                     .AndThen(new frc2::command::WaitCommand(0.1_s))
                     .Repeatedly())
      .OnTrue(new frc2::command::InstantCommand(
          [&]() { Leds::GetInstance().ready = true; }))
      .OnFalse(new frc2::command::InstantCommand(
          [&]() { Leds::GetInstance().ready = false; }));

  // Algae net score
  driver.LeftBumper()
      .And(shouldProcess.Negate())
      .And([&]() { return hasAlgae.value; })
      .And(driver.A())
      .WhileTrue(AlgaeScoreCommands::NetThrowScore(drive, superstructure)
                     .WithName("Algae Net Score"));

  // Algae eject
  driver.A()
      .And(driver.LeftBumper().Negate())
      .WhileTrue(
          superstructure->RunGoal(Superstructure::SuperstructureState::kToss)
              .WithName("Algae Toss"));

  // Strobe LEDs at human player
  driver.X().WhileTrue(new frc2::command::InstantCommand(
      [&]() { Leds::GetInstance().hpAttentionAlert = true; },
      [&]() { Leds::GetInstance().hpAttentionAlert = false; }));

  // Coral eject
  driver.B()
      .And(driver.RightBumper().Negate())
      .And(driver.RightTrigger().Negate())
      .WhileTrue(
          superstructure
              ->RunGoal(Superstructure::SuperstructureState::kGoodbyeCoralEject)
              .AlongWith(funnel->RunRoller(IntakeCommands::outtakeVolts)));

  // Force net
  driver.POVLeft().WhileTrue(
      superstructure->RunGoal(Superstructure::SuperstructureState::kThrown));

  // Force processor
  driver.POVRight().WhileTrue(
      superstructure->RunGoal(Superstructure::SuperstructureState::kProcessed));

  // Raise elevator
  driver.POVUp().ToggleOnTrue(
      superstructure->RunGoal(Superstructure::SuperstructureState::kL2Coral));

  // ***** OPERATOR CONTROLLER *****

  // Algae stow intake
  operatorController.LeftTrigger().WhileTrue(
      superstructure
          ->RunGoal(Superstructure::SuperstructureState::kAlgaeStowIntake)
          .WithName("Algae Stow Intake"));

  // Algae reef intake
  operatorController.POVDown().WhileTrue(superstructure->RunGoal(
      Superstructure::SuperstructureState::kAlgaeL2Intake));
  operatorController.POVUp().WhileTrue(superstructure->RunGoal(
      Superstructure::SuperstructureState::kAlgaeL3Intake));

  // Coral intake
  operatorController.RightBumper().WhileTrue(
      IntakeCommands::Intake(superstructure, funnel)
          .AlongWith(new frc2::command::InstantCommand(
              [&]() { superstructure->ResetHasCoral(); })));

  // Home elevator
  operatorController.LeftBumper().OnTrue(superstructure->RunHomingSequence());

  // Force net
  operatorController.POVLeft().WhileTrue(
      superstructure->RunGoal(Superstructure::SuperstructureState::kThrown));

  // Force processor
  operatorController.POVRight().WhileTrue(
      superstructure->RunGoal(Superstructure::SuperstructureState::kProcessed));

  // Algae eject
  operatorController.RightTrigger()
      .And(operatorController.A().Negate())
      .And(operatorController.B().Negate())
      .And(operatorController.X().Negate())
      .And(operatorController.Y().Negate())
      .WhileTrue(
          superstructure->RunGoal(Superstructure::SuperstructureState::kToss));

  // Operator commands for superstructure
  std::function<void(frc2::command::button::Trigger &,
                     FieldConstants::ReefLevel)>
      bindOperatorCoralScore = [&](frc2::command::button::Trigger &faceButton,
                                   FieldConstants::ReefLevel height) {
        faceButton.WhileTrueContinuous(
            superstructure
                ->RunGoal([&]() {
                  return Superstructure::GetScoringState(height, false);
                })
                .WithName("Operator Score on " +
                          FieldConstants::ToString(height)));
        faceButton.And(operatorController.RightTrigger())
            .WhileTrueContinuous(
                superstructure
                    ->RunGoal([&]() {
                      return Superstructure::GetScoringState(height, true);
                    })
                    .WithName("Operator Score & Eject On " +
                              FieldConstants::ToString(height)));
      };
  bindOperatorCoralScore(operatorController.A(),
                         FieldConstants::ReefLevel::kL1);
  bindOperatorCoralScore(operatorController.X(),
                         FieldConstants::ReefLevel::kL2);
  bindOperatorCoralScore(operatorController.B(),
                         FieldConstants::ReefLevel::kL3);
  bindOperatorCoralScore(operatorController.Y(),
                         FieldConstants::ReefLevel::kL4);

  // ***** MISCELANEOUS *****

  // Auto intake coral
  funnel->SetDefaultCommand(funnel->RunRoller([&]() {
    if (superstructure->IsRequestFunnelIntake()) {
      return IntakeCommands::funnelVolts();
    } else if (superstructure->IsRequestFunnelOuttake()) {
      return IntakeCommands::outtakeVolts();
    } else {
      return 0.0;
    }
  }));

  // Reset gyro
  auto driverStartAndBack = driver.Start().And(driver.Back());
  auto operatorStartAndBack =
      operatorController.Start().And(operatorController.Back());
  driverStartAndBack.Or(operatorStartAndBack)
      .OnTrue(new frc2::command::InstantCommand([&]() {
                RobotState::GetInstance().ResetPose(frc::Pose2d(
                    RobotState::GetInstance().GetEstimatedPose().Translation(),
                    AllianceFlipUtil::Apply(frc::Rotation2d::Degrees(0.0))));
              }).IgnoringDisable(true));

  // Superstructure coast
  superstructureCoast.OnTrue(new frc2::command::InstantCommand([&]() {
                               if (frc::DriverStation::IsDisabled()) {
                                 superstructureCoastOverride = true;
                                 Leds::GetInstance().superstructureCoast = true;
                               }
                             }).IgnoringDisable(true));
  superstructureCoast.OnFalse(new frc2::command::InstantCommand([&]() {
                                superstructureCoastOverride = false;
                                Leds::GetInstance().superstructureCoast = false;
                              }).IgnoringDisable(true));
  frc2::command::button::RobotModeTriggers::Disabled().OnFalse(
      new frc2::command::InstantCommand([&]() {
        superstructureCoastOverride = false;
        Leds::GetInstance().superstructureCoast = false;
      }).IgnoringDisable(true));

  // Endgame alerts
  frc2::command::button::Trigger([&]() {
    return frc::DriverStation::IsTeleopEnabled() &&
           frc::DriverStation::GetMatchTime() > 0.0 &&
           frc::DriverStation::GetMatchTime() <=
               std::round(endgameAlert1.Get());
  })
      .OnTrue(
          controllerRumbleCommand()
              ->WithTimeout(0.5)
              .BeforeStarting(
                  [&]() { Leds::GetInstance().endgameAlert = true; })
              .FinallyDo([&]() { Leds::GetInstance().endgameAlert = false; }));
  frc2::command::button::Trigger([&]() {
    return frc::DriverStation::IsTeleopEnabled() &&
           frc::DriverStation::GetMatchTime() > 0.0 &&
           frc::DriverStation::GetMatchTime() <=
               std::round(endgameAlert2.Get());
  })
      .OnTrue(controllerRumbleCommand()
                  ->WithTimeout(0.2)
                  .AndThen(new frc2::command::WaitCommand(0.1_s))
                  .Repeatedly()
                  .WithTimeout(0.9)
                  .BeforeStarting(
                      [&]() { Leds::GetInstance().endgameAlert = true; })
                  .FinallyDo([&]() {
                    Leds::GetInstance().endgameAlert = false;
                  })); // Rumble three times
}

frc2::command::Command *RobotContainer::controllerRumbleCommand() {
  return new frc2::command::InstantCommand(
      [&]() {
        driver.GetHID().SetRumble(frc::GenericHID::RumbleType::kBothRumble,
                                  1.0);
        operatorController.GetHID().SetRumble(
            frc::GenericHID::RumbleType::kBothRumble, 1.0);
      },
      [&]() {
        driver.GetHID().SetRumble(frc::GenericHID::RumbleType::kBothRumble,
                                  0.0);
        operatorController.GetHID().SetRumble(
            frc::GenericHID::RumbleType::kBothRumble, 0.0);
      });
}

void RobotContainer::updateDashboardOutputs() {
  frc::SmartDashboard::PutNumber("Match Time",
                                 frc::DriverStation::GetMatchTime());
}

void RobotContainer::updateAlerts() {
  // Controller disconnected alerts
  driverDisconnected.Set(
      !frc::DriverStation::IsJoystickConnected(driver.GetHID().GetPort()));
  operatorDisconnected.Set(!frc::DriverStation::IsJoystickConnected(
      operatorController.GetHID().GetPort()));
  overrideDisconnected.Set(!overrides.IsConnected());

  // AprilTag layout alert
  bool aprilTagAlertActive =
      getSelectedAprilTagLayout() != FieldConstants::defaultAprilTagType;
  aprilTagLayoutAlert.Set(aprilTagAlertActive);
  if (aprilTagAlertActive) {
    aprilTagLayoutAlert.SetText(
        "Non-default AprilTag layout in use (" +
        FieldConstants::ToString(getSelectedAprilTagLayout()) + ").");
  }
}

FieldConstants::AprilTagLayoutType RobotContainer::getSelectedAprilTagLayout() {
  if (aprilTagsReef.Get()) {
    if (frc::DriverStation::GetAlliance().value_or(
            frc::DriverStation::Alliance::kBlue) ==
        frc::DriverStation::Alliance::kBlue) {
      return FieldConstants::AprilTagLayoutType::kBlueReef;
    } else {
      return FieldConstants::AprilTagLayoutType::kRedReef;
    }
  } else if (aprilTagFieldBorder.Get()) {
    return FieldConstants::AprilTagLayoutType::kFieldBorder;
  } else {
    return FieldConstants::defaultAprilTagType;
  }
}

frc2::command::Command *RobotContainer::getAutonomousCommand() {
  return autoChooser.Get();
}
