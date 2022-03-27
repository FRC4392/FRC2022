/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.deceivers.util.JoystickHelper;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends CommandBase {
  public final Drivetrain mDrivetrain;
  public XboxController mController;
  private boolean lastScan;

  private JoystickHelper xHelper = new JoystickHelper(0);
  private JoystickHelper yHelper = new JoystickHelper(0);
  private JoystickHelper rotHelper = new JoystickHelper(0);
  
  public DriveCommand(Drivetrain Drivetrain, XboxController XboxController) {
    mDrivetrain = Drivetrain;
    mController = XboxController;
    
    addRequirements(mDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVel = 0;
    double yVel = 0;
    double rotVel = 0;

    yVel = mController.getLeftY();
    xVel = mController.getLeftX();

    if (Math.abs(yVel) < 0.09){
      yVel = 0;
    }

    if (Math.abs(xVel) < 0.09){
      xVel = 0;
    }

    rotVel = mController.getRightX();

    yVel = yHelper.setInput(yVel).applyDeadband(0.1).value;
    xVel = xHelper.setInput(xVel).applyDeadband(0.1).value;
    rotVel = rotHelper.setInput(rotVel).applyDeadband(0.1).value;
    
    if (mController.getRawButton(7) &! lastScan){
      mDrivetrain.resetGyro();
    }
    lastScan = mController.getRawButton(7);

    boolean fieldRelative = !mController.getRightBumper();
    mDrivetrain.drive(yVel, xVel, rotVel, fieldRelative);

    //mDrivetrain.setModulesAngle(xVel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
