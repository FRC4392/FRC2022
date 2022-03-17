// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AutoShootCommand extends CommandBase {
  private Shooter mShooter;
  private Limelight mLimelight;
  LinearFilter DistanceFilter = LinearFilter.movingAverage(5);
  LinearFilter AngleFilter = LinearFilter.movingAverage(2);
  /** Creates a new AutoShootCommand. */
  public AutoShootCommand(Shooter shooter, Limelight limelight) {
    mShooter = shooter;
    mLimelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {
      
      double limelightDistance = DistanceFilter.calculate(mLimelight.getDistanceToTarget().getAsDouble());
      double limeLightAngle = AngleFilter.calculate(mLimelight.getAngleOffset().getAsDouble());

      mShooter.setHood(mShooter.getHoodPositionForDistance(limelightDistance));
      mShooter.setPIDVelocity(mShooter.getShooterVelocityForDistance(limelightDistance));

      double targetAngle = limeLightAngle + mShooter.getAngle();

      if (Math.abs(targetAngle) > 95){
        targetAngle = 0;
      }
      mShooter.setTurretPosition(targetAngle);
      SmartDashboard.putNumber("AutoAimDistance", limelightDistance);
      SmartDashboard.putNumber("limelightAngle", targetAngle);
      SmartDashboard.putNumber("targetAngle", targetAngle);
      SmartDashboard.putNumber("wantedShotPower", mShooter.getShooterVelocityForDistance(limelightDistance));
    } catch (Exception e) {
      //TODO: handle exception
      e.printStackTrace();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.setHood(0);
    mShooter.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
