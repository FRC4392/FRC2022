// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class DumbAutoDrive extends CommandBase {
  Drivetrain mDrivetrain;
  double mStartPosition;
  Intake mIntake;
  double mStartTime;
  /** Creates a new DumbAutoDrive. */
  public DumbAutoDrive(Drivetrain drivetrain, double startPosition, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    mDrivetrain = drivetrain;
    mStartPosition = startPosition;
    mIntake = intake;

    addRequirements(mDrivetrain, mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrivetrain.setGyro(mStartPosition);
    mIntake.lower();
    mStartTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double driveSpeed = -.5;

    if (Timer.getFPGATimestamp() - mStartTime > 1){
      driveSpeed = 0;
    }
    mDrivetrain.drive(driveSpeed, 0, 0, false);
    mIntake.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivetrain.stop();
    mIntake.stop();
    mIntake.lift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - mStartTime > 2;
  }
}
