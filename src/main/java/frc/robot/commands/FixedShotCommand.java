// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FixedShotCommand extends CommandBase {
  Shooter mShooter;
  double mVelocity;
  double mHoodAngle;
  public FixedShotCommand(Shooter shooter, double velocity, double hoodAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooter = shooter;
    mVelocity = velocity;
    mHoodAngle = hoodAngle;

    addRequirements(shooter);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      mShooter.setPIDVelocity(mVelocity);
      mShooter.setHood(mHoodAngle);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      mShooter.setVelocity(0);
      mShooter.setHood(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
