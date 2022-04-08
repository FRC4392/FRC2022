// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCommand2 extends CommandBase {
  private final Intake mIntake;
  private final boolean liftAtEnd;

  /** Creates a new Intake. */
  public IntakeCommand2(Intake intake, boolean liftAtEnd) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intake;
    this.liftAtEnd = liftAtEnd;

    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntake.lower();
    mIntake.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (liftAtEnd){
      mIntake.lift();
    }
    mIntake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
