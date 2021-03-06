// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.NoSuchElementException;
import java.util.OptionalDouble;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private NetworkTable limelightTable;
  private NetworkTableEntry targetValid;
  private NetworkTableEntry targetAngle;
  private NetworkTableEntry targetHeight;
  private NetworkTableEntry ledSetting;
  private NetworkTableEntry camMode;
  private NetworkTableEntry snapshot;

  private double heightDifference = 102.625-39.6550;
  private double limelightAngle = 32.5;
  private Timer snapShotTimer = new Timer();


  public enum camMode {
    kVisionMode,
    kDriverMode
  }

  public enum LedMode {
    kOff,
    kBlink,
    kOn,
    kAuto
  }
  /** Creates a new Limelight. */
  public Limelight() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    targetValid = limelightTable.getEntry("tv");
    targetAngle = limelightTable.getEntry("tx");
    targetHeight = limelightTable.getEntry("ty");
    ledSetting = limelightTable.getEntry("ledMode");
    camMode = limelightTable.getEntry("camMode");
    snapshot = limelightTable.getEntry("snapshot");

  }

  public OptionalDouble getAngleOffset(){
    if (targetValid.getDouble(0.0) == 0){
      return OptionalDouble.empty();
    } else {
      double angleOffset = targetAngle.getDouble(100);
      if (angleOffset > 30){
        return OptionalDouble.empty();
      }
      return OptionalDouble.of(angleOffset);
    }
  }

  public OptionalDouble getDistanceToTarget(){
    if (targetValid.getDouble(0.0) == 0){
    return OptionalDouble.empty();
    } else{
      double heightAngle = targetHeight.getDouble(100);
      double angleOffset = targetAngle.getDouble(100);
      if (heightAngle > 30 || angleOffset > 30){
        return OptionalDouble.empty();
      } else {
        double distance = 0.0;
        distance = heightDifference/(Math.tan(Math.toRadians(heightAngle + limelightAngle))*Math.cos(Math.toRadians(angleOffset)));
        return OptionalDouble.of(distance);
      }
    }
  }

  public void setCamMode(camMode mode){
    switch (mode){
      case kVisionMode:
        camMode.setNumber(0);
        break;
      case kDriverMode:
        camMode.setNumber(1);
        break;
    }
  }

  public void setLEDMode(LedMode mode){
    switch (mode) {
      case kAuto:
        ledSetting.setNumber(0);
        break;
      case kOff:
        ledSetting.setNumber(1);
        break;
      case kBlink:
        ledSetting.setNumber(2);
        break;
      case kOn:
        ledSetting.setNumber(3);
    }
  }

  public void takeSnapshot(){
    snapshot.setNumber(1);
    snapShotTimer.reset();
    snapShotTimer.start();
  }

  public void stopSnapshot(){
    snapshot.setNumber(0);
    snapShotTimer.stop();
    snapShotTimer.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (snapShotTimer.get() > 1){
      stopSnapshot();
    }

    try {
      SmartDashboard.putNumber("limelightDistance", getDistanceToTarget().getAsDouble());
      SmartDashboard.putNumber("limelightAngle", getAngleOffset().getAsDouble());
    } catch (NoSuchElementException e) {
      //TODO: handle exception
    }
  }
}
