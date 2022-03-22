// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

  private CANSparkMax floor = new CANSparkMax(33, MotorType.kBrushless);
  /** Creates a new Conveyor. */
  public Conveyor() {
    floor.restoreFactoryDefaults();
    floor.setSmartCurrentLimit(30);
    floor.setInverted(false);
    floor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 65535);
    floor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
    floor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
  }

  public void setSpeed(double speed) {
    floor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
