// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class FollowPathPlannerPath extends CommandBase {
  Drivetrain drivetrain;
  boolean initPosition;
  PathPlannerTrajectory trajectory;
  double initTime;
  /** Creates a new FollowPathPlannerPath. */
  public FollowPathPlannerPath(PathPlannerTrajectory trajectory, boolean initPosition, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.initPosition = initPosition;
    this.trajectory = trajectory;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (initPosition){
      PathPlannerState initialState = trajectory.getInitialState();
      drivetrain.setLocation(initialState.poseMeters.getX(), initialState.poseMeters.getY(), initialState.holonomicRotation.getDegrees());
    }

    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.followPath(initTime, trajectory);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
