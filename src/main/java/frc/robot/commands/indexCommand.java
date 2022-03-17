// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sequencer;

public class indexCommand extends CommandBase {
  private Sequencer mSequencer;
  /** Creates a new indexCommand. */
  public indexCommand(Sequencer sequencer) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSequencer = sequencer;

    addRequirements(sequencer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mSequencer.getStartEye() && !mSequencer.getEndEye()){
      mSequencer.index();
    } else {
      mSequencer.stop();
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
