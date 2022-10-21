// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sequencer extends SubsystemBase {

  private CANSparkMax tower1 = new CANSparkMax(31, MotorType.kBrushless);
  private CANSparkMax tower2 = new CANSparkMax(32, MotorType.kBrushless);
  private DigitalInput startEye = new DigitalInput(1);
  private DigitalInput endEye = new DigitalInput(0);

  private final static double indexSpeed = 0.3;
  private final static double feedSpeed = .8;
  /** Creates a new Sequencer. */

  
    public Sequencer() {
      tower1.setInverted(false);
      tower1.setSmartCurrentLimit(30);
      tower1.setControlFramePeriodMs(50);
      tower1.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 65535);
      tower1.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
      tower1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
      
      tower2.setInverted(true);
      tower2.setSmartCurrentLimit(30);
      tower2.setControlFramePeriodMs(50);
      tower2.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 65535);
      tower2.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
      tower2.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    }

    public void setTowerSpeed(double speed){
      tower2.set(speed);
      tower1.set(speed);
    }
  
    public void index(){
      tower2.set(indexSpeed);
    }
  
    public void feed(){
      setTowerSpeed(feedSpeed);
    }

    public void reverse(){
      setTowerSpeed(-.25);
    }
  
    public void stop(){
      setTowerSpeed(0);
    }
  
    public boolean getStartEye(){
      return startEye.get();
    }
  
    public boolean getEndEye() {
      return endEye.get();
    }
  
    public void log(){
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
