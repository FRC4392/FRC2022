// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sequencer extends SubsystemBase {

  private CANSparkMax tower1 = new CANSparkMax(31, MotorType.kBrushless);
  private CANSparkMax tower2 = new CANSparkMax(32, MotorType.kBrushless);
  private CANSparkMax floor = new CANSparkMax(33, MotorType.kBrushless);
  private CANifier canifier = new CANifier(34);

  private final static double indexSpeed = 0.35;
  private final static double feedSpeed = .8;
  /** Creates a new Sequencer. */

  
    public Sequencer() {
    tower1.setSmartCurrentLimit(30);
      tower2.setSmartCurrentLimit(30);
      floor.setSmartCurrentLimit(30);
    }
  
    public void setSpeed(double speed) {
      floor.set(speed);
    }

    public void setTowerSpeed(double speed){
      tower2.set(speed);
      tower1.set(speed);
    }
  
    public void index(){
      setSpeed(indexSpeed);
    }
  
    public void feed(){
      setSpeed(feedSpeed);
    }
  
    public void stop(){
      setSpeed(0);
      setTowerSpeed(0);
    }
  
    public boolean getStartEye(){
      return !canifier.getGeneralInput(GeneralPin.SPI_MISO_PWM2P);
    }
  
    public boolean getEndEye() {
      return !canifier.getGeneralInput(GeneralPin.SPI_MOSI_PWM1P);
    }
  
    public void log(){
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
