// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private CANSparkMax climber1 = new CANSparkMax(41, MotorType.kBrushless);
  private CANSparkMax climber2 = new CANSparkMax(42, MotorType.kBrushless);
  private Solenoid pivot = new Solenoid(PneumaticsModuleType.REVPH, 1);
  private DigitalInput toplimitSwitch = new DigitalInput(2);


  /** Creates a new Climber. */
  public Climber() {
    climber1.restoreFactoryDefaults();
    climber2.restoreFactoryDefaults();
    climber1.setInverted(true);
    climber2.setInverted(false);

    climber1.setIdleMode(IdleMode.kBrake);
    climber2.setIdleMode(IdleMode.kBrake);

     climber1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
     climber2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    climber1.setSoftLimit(SoftLimitDirection.kReverse, -101);
    climber2.setSoftLimit(SoftLimitDirection.kReverse, -101);
    

    climber1.burnFlash();
    climber2.burnFlash();
  }
  
  public void setSpeed(double speed) {
    climber1.set(speed);
    climber2.set(speed);
    if(!(toplimitSwitch.get())){
      if(speed>0){
      climber1.set(0);
      climber2.set(0);
      }
    }
  }

  public void stop(){
    climber1.set(0);
    climber2.set(0);
}

public void lift(){
  climber1.setSoftLimit(SoftLimitDirection.kReverse, -101);
  climber2.setSoftLimit(SoftLimitDirection.kReverse, -101);
  pivot.set(false);
  }

public void lower(){
  climber1.setSoftLimit(SoftLimitDirection.kReverse, -150); //change max limit when we pivot
  climber2.setSoftLimit(SoftLimitDirection.kReverse, -150); 
  pivot.set(true);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
