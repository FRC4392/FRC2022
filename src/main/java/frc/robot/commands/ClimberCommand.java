// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.XboxController;

public class ClimberCommand extends CommandBase {
  public final Climber mClimber;
  public final JoystickButton mButton;
  public final XboxController mdriverController;
  public final POVButton mupClimb;
  public final POVButton mdownClimb;
  
  /** Creates a new Intake. */
  public ClimberCommand(Climber climber, JoystickButton button, XboxController driverController, POVButton upClimb, POVButton downClimb) {
    // Use addRequirements() here to declare subsystem dependencies.
    mClimber = climber;
    mButton = button;
    mdriverController = driverController;
    mupClimb = upClimb;
    mdownClimb = downClimb;
   // addRequirements(mIntake, mConveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   /* if(mupClimb == new POVButton(mdriverController, 0)){
        mClimber.lift();
    }
    if(mdownClimb == new POVButton(mdriverController, 180)){
      mClimber.lower();
  }*/
  mClimber.lift();
    if(mButton ==  new JoystickButton(mdriverController, 7)){
      mClimber.setSpeed(0.3);
  }
  else if(mButton ==  new JoystickButton(mdriverController, 8)){
    mClimber.setSpeed(-0.3);
}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     mClimber.setSpeed(0);
     mClimber.lower();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
