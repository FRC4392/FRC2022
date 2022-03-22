// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private CANSparkMax intake = new CANSparkMax(21, MotorType.kBrushless);
  private Solenoid pivot = new Solenoid(PneumaticsModuleType.REVPH, 0);
  private static final double intakeSpeed = 1;
  /** Creates a new Intake. */
  public Intake() {

    intake.setSmartCurrentLimit(20, 40, 5700);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 65535);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
  }

  public void setSpeed(double speed) {
    intake.set(speed);
  }
  
  public void intake(){
      setSpeed(intakeSpeed);
  } 
  
  public void stop(){
      intake.set(0);
  }
  
  public void lift(){
    pivot.set(false);
    }
  
  public void lower(){
    pivot.set(true);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
