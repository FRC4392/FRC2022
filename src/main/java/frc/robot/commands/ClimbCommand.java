// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class ClimbCommand extends CommandBase {
  /** Creates a new ClimbCommand. */
  Climber mClimber;
  Shooter mShooter;
  XboxController mController;

  public ClimbCommand(Climber climber, Shooter shooter, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    mClimber = climber;
    mShooter = shooter;
    mController = controller;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mShooter.setTurretPosition(0,0);
    mShooter.setVelocity(0);
    mShooter.setHood(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if(Math.abs(mController.getLeftY()) > 0.1){
    mClimber.setSpeed(mController.getLeftY());
  } else{
    mClimber.setSpeed(0);
  }

    if (mController.getRawButton(1)){
      mClimber.lower();
    } else {
      mClimber.lift(); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //mClimber.lift();
    mClimber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
