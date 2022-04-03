// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class HintTurretDirection extends CommandBase {
  Shooter mShooter;
  double mPosition;
  Drivetrain mDrivetrain;
  /** Creates a new HintTurretDirection. */
  public HintTurretDirection(Shooter shooter, double position, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooter = shooter;
    mPosition = position;
    mDrivetrain = drivetrain;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Field Relative   
   /* mPosition = mPosition + mDrivetrain.getRotation();
    if (Math.abs(mPosition) > 95)
    {
      mPosition = 0;
    }
    */

    mShooter.setTurretPosition(mPosition , 0);
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
