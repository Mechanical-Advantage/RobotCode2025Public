// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2025.commands;

import static org.littletonrobotics.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2025.Constants;
import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import org.littletonrobotics.frc2025.subsystems.drive.trajectory.HolonomicTrajectory;
import org.littletonrobotics.frc2025.subsystems.drive.trajectory.TrajectoryGenerationHelpers;
import org.littletonrobotics.frc2025.util.AllianceFlipUtil;
import org.littletonrobotics.frc2025.util.LoggedTunableNumber;
import org.littletonrobotics.frc2025.util.MirrorUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({TrajectoryGenerationHelpers.class})
public class DriveTrajectory extends Command {
  private static final LoggedTunableNumber linearkP =
      new LoggedTunableNumber("DriveTrajectory/LinearkP");
  private static final LoggedTunableNumber linearkD =
      new LoggedTunableNumber("DriveTrajectory/LinearkD");
  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("DriveTrajectory/ThetakP");
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("DriveTrajectory/ThetakD");
  private static final LoggedTunableNumber overrideMaxVelocity =
      new LoggedTunableNumber("DriveTrajectory/OverrideMaxVelocity", Units.degreesToRadians(360));
  private static final LoggedTunableNumber overrideMaxAcceleration =
      new LoggedTunableNumber(
          "DriveTrajectory/OverrideMaxAcceleration", Units.degreesToRadians(720));

  static {
    switch (Constants.getRobot()) {
      case COMPBOT, DEVBOT -> {
        linearkP.initDefault(8.0);
        linearkD.initDefault(0.0);
        thetakP.initDefault(4.0);
        thetakD.initDefault(0.0);
      }
      default -> {
        linearkP.initDefault(4.0);
        linearkD.initDefault(0.0);
        thetakP.initDefault(12.0);
        thetakD.initDefault(0.0);
      }
    }
  }

  private final Drive drive;
  private final HolonomicTrajectory trajectory;
  private final Timer timer = new Timer();
  private final Supplier<Pose2d> robotPose;
  private final boolean mirror;

  private Optional<Rotation2d> overrideRotation = Optional.empty();
  @AutoLogOutput private boolean isOverrideRotation = false;

  private final PIDController xController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(0.0, 0.0, 0.0);
  private final ProfiledPIDController overrideThetaController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));

  public DriveTrajectory(Drive drive, HolonomicTrajectory trajectory) {
    this(drive, trajectory, false);
  }

  public DriveTrajectory(Drive drive, HolonomicTrajectory trajectory, boolean mirror) {
    this(drive, trajectory, () -> RobotState.getInstance().getEstimatedPose(), mirror);
  }

  public DriveTrajectory(
      Drive drive, HolonomicTrajectory trajectory, Supplier<Pose2d> robot, boolean mirror) {
    this.drive = drive;
    this.trajectory = trajectory;
    robotPose = robot;
    this.mirror = mirror;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    overrideThetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    timer.restart();

    xController.reset();
    yController.reset();
    thetaController.reset();

    Logger.recordOutput(
        "Trajectory/TrajectoryPoses",
        Arrays.stream(trajectory.getTrajectoryPoses())
            .map(pose -> AllianceFlipUtil.apply(mirror ? MirrorUtil.apply(pose) : pose))
            .toArray(Pose2d[]::new));
  }

  @Override
  public void execute() {
    // Update tunable numbers
    if (linearkP.hasChanged(hashCode()) || linearkD.hasChanged(hashCode())) {
      xController.setPID(linearkP.get(), 0.0, linearkD.get());
      yController.setPID(linearkP.get(), 0.0, linearkD.get());
    }
    if (thetakP.hasChanged(hashCode()) || thetakD.hasChanged(hashCode())) {
      thetaController.setPID(thetakP.get(), 0.0, thetakD.get());
      overrideThetaController.setPID(thetakP.get(), 0.0, thetakD.get());
    }
    if (overrideMaxVelocity.hasChanged(hashCode())
        || overrideMaxAcceleration.hasChanged(hashCode())) {
      overrideThetaController.setConstraints(
          new TrapezoidProfile.Constraints(
              overrideMaxVelocity.get(), overrideMaxAcceleration.get()));
    }

    Pose2d robot = robotPose.get();
    // Get setpoint state and flip
    VehicleState setpointState =
        AllianceFlipUtil.apply(
            mirror
                ? MirrorUtil.apply(trajectory.sample(timer.get()))
                : trajectory.sample(timer.get()));

    // Calculate feedback
    double xFeedback = xController.calculate(robot.getX(), setpointState.getX());
    double yFeedback = yController.calculate(robot.getY(), setpointState.getY());
    double thetaFeedback =
        thetaController.calculate(robot.getRotation().getRadians(), setpointState.getTheta());

    // Calculate module forces
    var moduleForces =
        setpointState.getModuleForcesList().stream()
            .map(
                forces ->
                    new Translation2d(forces.getFx(), forces.getFy())
                        .rotateBy(Rotation2d.fromRadians(setpointState.getTheta()).unaryMinus())
                        .toVector())
            .toList();

    // Calculate final omega using override rotation if present
    final double finalOmega =
        overrideRotation
            .map(
                setpoint ->
                    overrideThetaController.calculate(
                        robot.getRotation().getRadians(), setpoint.getRadians()))
            .orElse(thetaFeedback + setpointState.getOmega());

    // Command drive
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFeedback + setpointState.getVx(),
            yFeedback + setpointState.getVy(),
            finalOmega,
            Rotation2d.fromDegrees(robot.getRotation().getDegrees())),
        moduleForces);

    // Log outputs
    Logger.recordOutput("Trajectory/RobotPose", robot);
    Logger.recordOutput("Trajectory/SetpointPose", setpointState.getPose());
    Logger.recordOutput(
        "Trajectory/Feedback",
        new Pose2d(xFeedback, yFeedback, Rotation2d.fromRadians(thetaFeedback)));
    Logger.recordOutput(
        "Trajectory/VelocityFeedforward",
        new Pose2d(
            setpointState.getVx(),
            setpointState.getVy(),
            Rotation2d.fromRadians(setpointState.getOmega())));
    Logger.recordOutput(
        "Trajectory/OverrideRotation",
        overrideRotation
            .map(
                rotation ->
                    new Pose2d(
                        setpointState.getPose().getTranslation(),
                        Rotation2d.fromRadians(overrideThetaController.getSetpoint().position)))
            .orElse(Pose2d.kZero));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return timer.get() >= trajectory.getDuration();
  }

  public void setOverrideRotation(Optional<Rotation2d> rotation) {
    overrideRotation = rotation;
    if (!isOverrideRotation) {
      overrideThetaController.reset(
          robotPose.get().getRotation().getRadians(),
          RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond);
    }
    isOverrideRotation = rotation.isPresent();
  }
}
