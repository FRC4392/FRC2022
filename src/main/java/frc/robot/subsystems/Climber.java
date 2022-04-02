// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private CANSparkMax climber1 = new CANSparkMax(41, MotorType.kBrushless);
  private CANSparkMax climber2 = new CANSparkMax(42, MotorType.kBrushless);
  private Solenoid pivot = new Solenoid(PneumaticsModuleType.REVPH, 1);

  /** Creates a new Climber. */
  public Climber() {
    climber1.restoreFactoryDefaults();
    climber2.restoreFactoryDefaults();
    climber1.setInverted(true);
    climber2.setInverted(false);

    climber1.setIdleMode(IdleMode.kBrake);
    climber2.setIdleMode(IdleMode.kBrake);

    climber1.burnFlash();
    climber2.burnFlash();
  }
  
  
  public void setSpeed(double speed) {
    climber1.set(speed);
    climber2.set(speed);
  }
  public void stop(){
    climber1.set(0);
    climber2.set(0);
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
