// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    private final SwerveModuleV3 Module1 = new SwerveModuleV3(mAzimuth1, mDriveMotor1, new Translation2d(0.259999988, 0.2346), 3, 0.000593, "Module 1"); //002632 0005867
    private final SwerveModuleV3 Module2 = new SwerveModuleV3(mAzimuth2, mDriveMotor2, new Translation2d(0.259999988, -0.2346), 4, 0.001790, "Module 2"); //00380 001793
    private final SwerveModuleV3 Module3 = new SwerveModuleV3(mAzimuth3, mDriveMotor3, new Translation2d(-0.259999988, -0.2346), 5, 0.003690, "Module 3"); //001155 002707
    private final SwerveModuleV3 Module4 = new SwerveModuleV3(mAzimuth4, mDriveMotor4, new Translation2d(-0.259999988,  0.2346), 6, 0.002667, "Module 4"); //00063 002641

    private final SwerveDrive mSwerveDrive = new SwerveDrive(this::getRotation, Module1, Module2, Module3, Module4);

    private double gyroOffset = 0;

  public Drivetrain() {
    pidgey.setFusedHeading(0);
    mDriveMotor1.enableVoltageCompensation(11);
    mDriveMotor2.enableVoltageCompensation(11);
    mDriveMotor3.enableVoltageCompensation(11);
    mDriveMotor4.enableVoltageCompensation(11);

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
  
    public void followPath(double initTime, PathPlannerTrajectory pptrajectory){
      mSwerveDrive.followPath(initTime, pptrajectory);
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
  
    //derek
    public ChassisSpeeds getSpeeds(){
      ChassisSpeeds chassisSpeeds = mSwerveDrive.getChassisSpeeds();
      return(chassisSpeeds);
    }

    public void resetGyro(){
      setGyro(0);
    }

    public void setGyro(double position){
      gyroOffset = (position - pidgey.getFusedHeading());
    }
  
    public double getRotation() {
      return pidgey.getFusedHeading() + gyroOffset;
    }
  
    public double getYaw() {
      return pidgey.getYaw();
    }
    public void setStartPosition(){
      mSwerveDrive.setStartPostion();
    }

    public void setModulesAngle(double angle, int module){
      mSwerveDrive.setModulesAngle(angle, module);
    }
}
