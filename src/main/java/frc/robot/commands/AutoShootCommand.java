// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {

     ChassisSpeeds speeds = mDrivetrain.getSpeeds(); //gets the .toChassisSpeeds
   double rpmtoft = 45.0/4000.0;

     //misguided method
   /*  double xVel = 0;
     double yVel = 0;
     double rotVel = 0;
     yVel = mController.getLeftY(); 
     xVel = mController.getLeftX(); 
     rotVel = mController.getRightX()*2.5; //unused, 2.5 is in Drivetrain subsystem so I brought it here

     //Getting robot velocities w/ respect to field
     speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, Rotation2d.fromDegrees(mGyroAngle.getAsDouble()));
*/

      //limelight distance and angle to hub
     double limelightDistance = DistanceFilter.calculate(mLimelight.getDistanceToTarget().getAsDouble());
     double limeLightAngle = AngleFilter.calculate(mLimelight.getAngleOffset().getAsDouble());

     rpmtoft = rpmtoft*Math.cos(mShooter.getHoodPositionForDistance(limelightDistance));

     //net robot velocity, angle, and shooter angular velocity converted to m/s
     double vnet = -Math.sqrt((speeds.vxMetersPerSecond*speeds.vxMetersPerSecond)+(speeds.vyMetersPerSecond*speeds.vyMetersPerSecond));
     double anglenet = -Math.atan2(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond); //negative sign because i'm too lazy to put it into the math
     double shootVel = mShooter.getShooterVelocityForDistance(limelightDistance) * rpmtoft * (0.3048); //.3048 is ft/s to m/s, 12.3/4000 is from julia calculator on how fast the ball should be going when we shoot

     //law of cosines to find unknowns of velocity triangle
     double vdesired = Math.sqrt((shootVel * shootVel) + (vnet*vnet) - (2.0 * shootVel * vnet * Math.cos(anglenet+(Math.PI/2.0))));
     double adjustment = Math.acos(((vnet*vnet)-(shootVel*shootVel)-(vdesired*vdesired))/(-2.0*shootVel*vdesired));

     if(speeds.vyMetersPerSecond > 0.0 ){
       adjustment = -adjustment;
     }
     //double targetAngle = limeLightAngle + mShooter.getAngle();
     double targetAngle = limeLightAngle + mShooter.getAngle() + ((adjustment*180.0/Math.PI)*27.0);
     
     //Set Shooter Values, 
      mShooter.setHood(mShooter.getHoodPositionForDistance(mShooter.getDistanceFromShooterVelocity(vdesired*(1.0/rpmtoft)*(1.0/.3048))));
      mShooter.setPIDVelocity(vdesired*(1.0/rpmtoft)*(1.0/.3048));

      //mShooter.setPIDVelocity(mShooter.getShooterVelocityForDistance(limelightDistance));
      //mShooter.setHood(mShooter.getHoodPositionForDistance(limelightDistance));

      if (Math.abs(targetAngle) > 95.0){
        targetAngle = 0;
      }

      mShooter.setTurretPosition(targetAngle);

      //shoot on the move numbers
      SmartDashboard.putNumber("MoveShotPower", vdesired*(4000/12.3)*(1/.3048));
      SmartDashboard.putNumber("MoveAdjustment", (adjustment*180.0/Math.PI)*27.0);

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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
