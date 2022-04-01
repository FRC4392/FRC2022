// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.Shooter;

public class AutoEjectCommand extends CommandBase {
  private Sequencer mSequencer;
  private Shooter mShooter;
  /** Creates a new AutoEjectCommand. */
  public AutoEjectCommand(Sequencer tower, Shooter shooter) {
    mSequencer = tower;
    mShooter = shooter;
    addRequirements(mSequencer, mShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mShooter.setTurretPosition(0);
    mShooter.setHood(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.setPIDVelocity(600);
    mSequencer.feed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.setHood(0);
    mSequencer.stop();
    mShooter.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
