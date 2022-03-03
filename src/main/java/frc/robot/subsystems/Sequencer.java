// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sequencer extends SubsystemBase {

  private CANSparkMax tower1 = new CANSparkMax(31, MotorType.kBrushless);
  private CANSparkMax tower2 = new CANSparkMax(32, MotorType.kBrushless);
  private CANSparkMax floor = new CANSparkMax(33, MotorType.kBrushless);
  /** Creates a new Sequencer. */
  public Sequencer() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
