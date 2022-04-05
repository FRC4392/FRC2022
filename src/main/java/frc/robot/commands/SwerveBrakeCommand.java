// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SwerveBrakeCommand extends CommandBase {
  private Drivetrain mDrivetrain;
  /** Creates a new SwerveBrakeCommand. */
  public SwerveBrakeCommand(Drivetrain drivetrain) {
    mDrivetrain = drivetrain;

    addRequirements(mDrivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = 45;
    for (int i = 0; i < 4; i++) {
      mDrivetrain.setModulesAngle(angle, i);
      angle += 90;
    }
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
