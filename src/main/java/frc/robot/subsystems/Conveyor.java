// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {

  private CANSparkMax floor = new CANSparkMax(33, MotorType.kBrushless);
  /** Creates a new Conveyor. */
  public Conveyor() {
    floor.restoreFactoryDefaults();
    floor.setSmartCurrentLimit(30);
    floor.setInverted(true);
  }

  public void setSpeed(double speed) {
    floor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
