// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveModuleV3;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

    private final CANSparkMax mDriveMotor1 = new CANSparkMax(11, MotorType.kBrushless);
    private final CANSparkMax mDriveMotor2 = new CANSparkMax(13, MotorType.kBrushless);
    private final CANSparkMax mDriveMotor3 = new CANSparkMax(15, MotorType.kBrushless);
    private final CANSparkMax mDriveMotor4 = new CANSparkMax(17, MotorType.kBrushless);

    private final CANSparkMax mAzimuth1 = new CANSparkMax(12, MotorType.kBrushless);
    private final CANSparkMax mAzimuth2 = new CANSparkMax(14, MotorType.kBrushless);
    private final CANSparkMax mAzimuth3 = new CANSparkMax(16, MotorType.kBrushless);
    private final CANSparkMax mAzimuth4 = new CANSparkMax(18, MotorType.kBrushless);

    private final PigeonIMU pidgey = new PigeonIMU(10);

    private final SwerveModuleV3 Module1 = new SwerveModuleV3(mAzimuth1, mDriveMotor1, new Translation2d(0.259999988, 0.2346), "Module 1");
    private final SwerveModuleV3 Module2 = new SwerveModuleV3(mAzimuth2, mDriveMotor2, new Translation2d(0.259999988, -0.2346), "Module 2");
    private final SwerveModuleV3 Module3 = new SwerveModuleV3(mAzimuth3, mDriveMotor3, new Translation2d(-0.259999988, -0.2346), "Module 3");
    private final SwerveModuleV3 Module4 = new SwerveModuleV3(mAzimuth4, mDriveMotor4, new Translation2d(-0.259999988,  0.2346), "Module 4");

    private final SwerveDrive mSwerveDrive = new SwerveDrive(pidgey::getFusedHeading, Module1, Module2, Module3, Module4);

  public Drivetrain() {
    pidgey.setFusedHeading(0);
    //setLocation(3.892, 1.295, 0 ); //Slalom
    //setLocation(1.1661376518218622, 2.2212145748987857, 0 ); //Barrel Run
    setStartPosition();
  }

    public void drive(double forward, double strafe, double azimuth, boolean fieldRelative){
      azimuth = azimuth*2.5;
      mSwerveDrive.drive(forward, strafe, azimuth, fieldRelative);
    }
  
    public void driveClosedLoop(double forward, double strafe, double azimuth, boolean fieldRelative){
      if (!fieldRelative){
        forward = -forward;
        strafe = -strafe;
      }
      azimuth = azimuth*2.5;
      mSwerveDrive.driveClosedLoop(forward, strafe, azimuth, fieldRelative);
    }
  
    public void stop(){
      mSwerveDrive.stop();
    }
  
    public void followPath(double initTime){
      mSwerveDrive.followPath(initTime);
    }
  
    @Override
    public void periodic() {
      mSwerveDrive.updateOdometry();
      mSwerveDrive.log();
      SmartDashboard.putNumber("GyroAbs", pidgey.getFusedHeading());
    }
  
    public void setLocation(double x, double y, double angle){
      mSwerveDrive.setLocation(x, y, angle);
  
    }
  
    public void resetGyro(){
      pidgey.setFusedHeading(0);
    }
  
    public double getRotation() {
      return pidgey.getFusedHeading();
    }
  
    public void setStartPosition(){
      mSwerveDrive.setStartPostion();
    }

    public void setModulesAngle(double angle){
      mSwerveDrive.setModulesAngle(angle);
    }
}
