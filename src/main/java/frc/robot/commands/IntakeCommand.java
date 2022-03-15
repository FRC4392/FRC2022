// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sequencer;

public class IntakeCommand extends CommandBase {
  public final Intake mIntake;
  public final Conveyor mConveyor;
  public final Sequencer mSequencer;

  /** Creates a new Intake. */
  public IntakeCommand(Intake intake, Conveyor conveyor, Sequencer sequencer) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intake;
    mConveyor = conveyor;
    mSequencer = sequencer;
    addRequirements(mIntake, mConveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntake.lower();
    mIntake.intake();
    mConveyor.setSpeed(1);
    mSequencer.setTowerSpeed(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.lift();
    mIntake.stop();
    mConveyor.setSpeed(0);
    mSequencer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
