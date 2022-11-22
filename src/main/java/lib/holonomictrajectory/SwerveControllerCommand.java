// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.holonomictrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SwerveControllerCommand extends CommandBase {

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  private final CustomHolonomicDriveController controller;

  private final Supplier<Pose2d> pose;
  private final Consumer<ChassisSpeeds> output;
  private final Timer timer = new Timer();

  private final CustomTrajectoryGenerator customGenerator = new CustomTrajectoryGenerator();

  public SwerveControllerCommand(
      List<Waypoint> waypoints,
      TrajectoryConfig config,
      Supplier<Pose2d> pose,
      PIDController xController,
      PIDController yController,
      PIDController thetaController,
      Consumer<ChassisSpeeds> output,
      Subsystem... requirements) {
    this.pose = pose;
    this.xController = xController;
    this.yController = yController;
    this.thetaController = thetaController;
    this.controller =
        new CustomHolonomicDriveController(
            this.xController, this.yController, this.thetaController);
    this.output = output;

    addRequirements(requirements);

    try {
      customGenerator.generate(config, waypoints);
    } catch (TrajectoryGenerationException e) {
      DriverStation.reportError("Failed to generate trajectory.", e.getStackTrace());
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    //    thetaController.reset(pose.get().getRotation().getRadians());

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State driveState = customGenerator.getDriveTrajectory().sample(timer.get());
    RotationSequence.State holonomicRotationState =
        customGenerator.getHolonomicRotationSequence().sample(timer.get());

    ChassisSpeeds nextDriveState =
        controller.calculate(pose.get(), driveState, holonomicRotationState);
    output.accept(nextDriveState);

    Logger.getInstance()
        .recordOutput(
            "Odometry/ProfileSetpoint",
            new double[] {
              driveState.poseMeters.getX(),
              driveState.poseMeters.getY(),
              holonomicRotationState.position.getRadians()
            });
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    output.accept(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(customGenerator.getDriveTrajectory().getTotalTimeSeconds());
  }
}
