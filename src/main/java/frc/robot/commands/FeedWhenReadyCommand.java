// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.Shooter;

public class FeedWhenReadyCommand extends CommandBase {
  private Sequencer mSequencer;
  private Shooter mShooter;
  private Conveyor conveyor;
  /** Creates a new FeedWhenReadyCommand. */
  public FeedWhenReadyCommand(Sequencer sequencer, Shooter shooter, Conveyor conveyor) {
    mSequencer = sequencer;
    mShooter = shooter;
    this.conveyor = conveyor;
    addRequirements(mSequencer, conveyor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mShooter.isReady()){
      conveyor.setSpeed(1);
      mSequencer.feed();
    } else{
      mSequencer.stop();
      conveyor.setSpeed(.5);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mSequencer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
