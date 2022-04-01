// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
  
  double rumbleAngle = 0;

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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {

     ChassisSpeeds speeds = mDrivetrain.getSpeeds(); //gets the .toChassisSpeeds

     double rpmtoft = 25.0 / 4000.0; //guess and check number, higher numbers means the robot speeds affects the shot less, and vice versa
     double turretFactor = 12.0; //turret angle guess and check number, higher numbers makes the adjusted turret angle larger

     double radtodeg = 180.0 / Math.PI;
     double degtorad = 1.0 / radtodeg;
     double ftToM = 0.3048;
     double mToFt = 1/ftToM;

      //limelight distance and angle to hub
      double limelightDistance = DistanceFilter.calculate(mLimelight.getDistanceToTarget().getAsDouble());
      double limeLightAngle = AngleFilter.calculate(mLimelight.getAngleOffset().getAsDouble());

     //changes robot velocities to perpendicular and parallel velocities to the goal, matrix is written out because this isn't matlab
      double angleToGoal = limeLightAngle + mShooter.getAngle();
      double robotAngle = (mDrivetrain.getRotation() + angleToGoal) * degtorad;
      speeds.vxMetersPerSecond = speeds.vxMetersPerSecond*Math.cos(robotAngle) - speeds.vyMetersPerSecond*Math.sin(robotAngle); //rotation matrix
      speeds.vyMetersPerSecond = speeds.vxMetersPerSecond*Math.sin(robotAngle) + speeds.vyMetersPerSecond*Math.cos(robotAngle);

     //adjust straight shot velocity by hood angle, angle goes from 20-45 degrees, math is a function that puts 45 degrees
     // when getHoodPosition is at 1, 20 degrees when 0
     rpmtoft = rpmtoft * Math.cos((mShooter.getHoodPositionForDistance(limelightDistance) * 25 * degtorad) + (20 * degtorad));
      //might be unneeded

     //net robot velocity, angle, and shooter angular velocity converted to m/s
     double vnet = -Math.sqrt((speeds.vxMetersPerSecond*speeds.vxMetersPerSecond)+(speeds.vyMetersPerSecond*speeds.vyMetersPerSecond));
     double anglenet = -Math.atan2(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
     double shootVel = mShooter.getShooterVelocityForDistance(limelightDistance) * rpmtoft * ftToM;

     //law of cosines to find unknowns of velocity triangle
     double vdesired = Math.sqrt((shootVel * shootVel) + (vnet*vnet) - (2.0 * shootVel * vnet * Math.cos(anglenet+(Math.PI/2.0))));
     double adjustment = Math.acos(((vnet*vnet)-(shootVel*shootVel)-(vdesired*vdesired))/(-2.0*shootVel*vdesired));

     if(speeds.vyMetersPerSecond > 0.0 ){
       adjustment = -adjustment;
     }

     double targetAngle = limeLightAngle + mShooter.getAngle() + ((adjustment*radtodeg) * turretFactor);// + speeds.omegaRadiansPerSecond * timeInterval;
     
     //Set Shooter Values, 
      mShooter.setHood(mShooter.getHoodPositionForDistance(mShooter.getDistanceFromShooterVelocity(vdesired * (1.0/rpmtoft) * mToFt)));
      mShooter.setPIDVelocity(vdesired * (1.0/rpmtoft) * mToFt);

      if (Math.abs(targetAngle) > 95.0){
        targetAngle = 0;
      }
      mShooter.setTurretPosition(targetAngle); 
      
      rumbleAngle = limeLightAngle - mShooter.getAngle();
      mController.setRumble(RumbleType.kLeftRumble, Math.cos(rumbleAngle * degtorad));
      mController.setRumble(RumbleType.kRightRumble, Math.sin(rumbleAngle * degtorad));
        
      //shoot on the move numbers
      SmartDashboard.putNumber("MoveShotPower", vdesired * rpmtoft * mToFt);
      SmartDashboard.putNumber("MoveAdjustment", (adjustment*radtodeg)*12.0);
      SmartDashboard.putNumber("Vx",speeds.vxMetersPerSecond);
      SmartDashboard.putNumber("Vy",speeds.vyMetersPerSecond);
      SmartDashboard.putNumber("vNet",vnet);
      SmartDashboard.putNumber("AutoAimDistance", limelightDistance);
      SmartDashboard.putNumber("limelightAngle", targetAngle);
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
    mController.setRumble(RumbleType.kLeftRumble, 0);
    mController.setRumble(RumbleType.kRightRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
