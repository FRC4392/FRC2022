// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private TalonFX shooter1 = new TalonFX(51);
  private TalonFX shooter2 = new TalonFX(52);
  private CANSparkMax hood = new CANSparkMax(53, MotorType.kBrushless);
  private CANSparkMax turret = new CANSparkMax(54, MotorType.kBrushless);
  private CANifier canifier = new CANifier(55);
  /** Creates a new Shooter. */
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
