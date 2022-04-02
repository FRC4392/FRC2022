// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight.LedMode;
import frc.robot.subsystems.Drivetrain;

public class AutoShootCommand extends CommandBase {
  private Shooter mShooter;
  private Limelight mLimelight;
  private Drivetrain mDrivetrain;
  LinearFilter DistanceFilter = LinearFilter.movingAverage(5);
  LinearFilter AngleFilter = LinearFilter.movingAverage(2);
  
  public XboxController mController;

  /** Creates a new AutoShootCommand. */
  public AutoShootCommand(Shooter shooter, Limelight limelight, XboxController XboxController, Drivetrain drivetrain) {
    mShooter = shooter;
    mLimelight = limelight;
    mDrivetrain = drivetrain;
    mController = XboxController;

    addRequirements(mShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mLimelight.setLEDMode(LedMode.kOn);
  }
double oldAdjustment = 0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {

     ChassisSpeeds speeds = mDrivetrain.getSpeeds(); //gets the .toChassisSpeeds

     double rpmtoft = 20.0 / 4000.0; //guess and check number, higher numbers means the robot speeds affects the shot more, and vice versa
     double turretFactor = 2.0; //turret angle guess and check number, higher numbers makes the adjusted turret angle larger
     
     rpmtoft = mShooter.createSmartDashboardNumber("RPM Factor", 12.3)/4000.0;
     turretFactor = mShooter.createSmartDashboardNumber("Turret Factor", 5.0);

     double radtodeg = 180.0 / Math.PI;
     double degtorad = 1.0 / radtodeg;
     double ftToM = 0.3048;
     double mToFt = 1.0 / ftToM;

      //limelight distance and angle to hub
      double limelightDistance = DistanceFilter.calculate(mLimelight.getDistanceToTarget().getAsDouble());
      double limeLightAngle = AngleFilter.calculate(mLimelight.getAngleOffset().getAsDouble());
      
      SmartDashboard.putNumber("Prev Vx",speeds.vxMetersPerSecond*mToFt);
      SmartDashboard.putNumber("Prev Vx",speeds.vyMetersPerSecond*mToFt);

      double angleToGoal = (limeLightAngle + mShooter.getAngle()); //- oldAdjustment;
      SmartDashboard.putNumber("Angle To Goal",angleToGoal*radtodeg);

      //changes robot velocities to perpendicular and parallel velocities to the goal, matrix is written out because this isn't matlab
      //either math is wrong or input angle is wrong
      
      //speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, Rotation2d.fromDegrees(angleToGoal));

     // speeds.vxMetersPerSecond = speeds.vxMetersPerSecond*Math.cos(-angleToGoal) - speeds.vyMetersPerSecond*Math.sin(-(angleToGoal)); //rotation matrix
     // speeds.vyMetersPerSecond = speeds.vxMetersPerSecond*Math.sin(-angleToGoal) + speeds.vyMetersPerSecond*Math.cos(-(angleToGoal));

     //adjust straight shot velocity by hood angle, angle goes from 20-45 degrees, math is a function that puts 45 degrees
     // when getHoodPosition is at 1, 20 degrees when 0
     rpmtoft = rpmtoft * Math.cos((mShooter.getHoodPositionForDistance(limelightDistance) * (-35) * degtorad) + ((70) * degtorad));

     //net robot velocity, angle, and shooter angular velocity converted to m/s
     double vnet = -Math.sqrt((speeds.vxMetersPerSecond*speeds.vxMetersPerSecond)+(speeds.vyMetersPerSecond*speeds.vyMetersPerSecond));
     double anglenet = -Math.atan2(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
     double shootVel = mShooter.getShooterVelocityForDistance(limelightDistance) * rpmtoft * ftToM;

     //law of cosines to find unknowns of velocity triangle
     double vdesired = Math.sqrt((shootVel * shootVel) + (vnet*vnet) - (2.0 * shootVel * vnet * Math.cos(anglenet+(Math.PI/2.0))));
     double adjustment = Math.acos(((vnet*vnet)-(shootVel*shootVel)-(vdesired*vdesired))/(-2.0*shootVel*vdesired));
    
     //make adjustment negative, depending on what direction we're moving
     if(speeds.vyMetersPerSecond > 0.0 ){
       adjustment = -adjustment;
     }
     oldAdjustment = adjustment;

     double targetAngle = limeLightAngle + mShooter.getAngle() + ((adjustment*radtodeg) * turretFactor);// + speeds.omegaRadiansPerSecond * timeInterval;
     
     //Set Shooter Values, 
      mShooter.setHood(mShooter.getHoodPositionForDistance(mShooter.getDistanceFromShooterVelocity(vdesired * (1.0/rpmtoft) * mToFt)));
      mShooter.setPIDVelocity(vdesired * (1.0/rpmtoft) * mToFt);

      if (Math.abs(targetAngle) > 100.0){
        targetAngle = 0;
      }
      mShooter.setTurretPosition(targetAngle, speeds.omegaRadiansPerSecond); 
      
      SmartDashboard.putNumber("romtoft",rpmtoft);

      SmartDashboard.putNumber("MoveShotPower", vdesired * rpmtoft * mToFt);
      SmartDashboard.putNumber("MoveAdjustment", (adjustment * radtodeg) * turretFactor);
      SmartDashboard.putNumber("Vx",speeds.vxMetersPerSecond * mToFt);
      SmartDashboard.putNumber("Vy",speeds.vyMetersPerSecond * mToFt);

      SmartDashboard.putNumber("AutoAimDistance", limelightDistance);
      SmartDashboard.putNumber("limelightAngle", limeLightAngle);
      SmartDashboard.putNumber("targetAngle", targetAngle);
      SmartDashboard.putNumber("wantedShotPower", mShooter.getShooterVelocityForDistance(limelightDistance));
    } catch (Exception e) {
      e.printStackTrace();
      mShooter.setTurretSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.setHood(0);
    mShooter.setVelocity(0);
    mLimelight.setLEDMode(LedMode.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
