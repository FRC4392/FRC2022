// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Sequencer;

public class AutoFeedShooter extends CommandBase {
  private Sequencer sequencer;
  private Conveyor conveyor;
  private double initTime;
  /** Creates a new AutoFeedShooter. */
  public AutoFeedShooter(Sequencer sequencer, Conveyor conveyor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sequencer = sequencer;
    this.conveyor = conveyor;

    addRequirements(sequencer);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - initTime > .75){
      sequencer.feed();
      conveyor.setSpeed(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sequencer.stop();
    conveyor.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
