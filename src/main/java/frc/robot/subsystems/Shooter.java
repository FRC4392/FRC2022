// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;

public class Shooter extends SubsystemBase {

  private TalonFX shooter1 = new TalonFX(51);
  private TalonFX shooter2 = new TalonFX(52);
  private CANSparkMax hood = new CANSparkMax(53, MotorType.kBrushless);
  private CANSparkMax turret = new CANSparkMax(54, MotorType.kBrushless);
  private RelativeEncoder hoodEncoder;
  private SparkMaxPIDController hoodPID;
  private RelativeEncoder turretEncoder;
  private SparkMaxPIDController turretPID;
  private CANifier canifier = new CANifier(55);
  private CANCoder cancoder = new CANCoder(56);
  private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodMap = new InterpolatingTreeMap<>();
  static {
    hoodMap.put(new InterpolatingDouble(62.0), new InterpolatingDouble(0.0));
    hoodMap.put(new InterpolatingDouble(407.0), new InterpolatingDouble(1.0));
  }
  private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> speedMap = new InterpolatingTreeMap<>();
  static {
    speedMap.put(new InterpolatingDouble(62.0), new InterpolatingDouble(2300.0));
    speedMap.put(new InterpolatingDouble(240.0), new InterpolatingDouble(2650.0));
    speedMap.put(new InterpolatingDouble(407.0), new InterpolatingDouble(3420.0));
  }

  private double setpoint = 0;
  /** Creates a new Shooter. */
  public Shooter() {  
    
    shooter1.setInverted(true);
    shooter1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shooter1.config_kF(0, 0.055);
    shooter1.config_kP(0, 0.1);
    shooter1.configPeakOutputReverse(0);
    shooter1.configPeakOutputForward(1);
    shooter1.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);

    shooter2.setInverted(false);
    shooter2.follow(shooter1);

    hood.setIdleMode(IdleMode.kBrake);
    hood.setSoftLimit(SoftLimitDirection.kReverse, 0.0f);
    hood.setSoftLimit(SoftLimitDirection.kForward, 1.0f);
    hoodEncoder = hood.getEncoder();

    hoodEncoder.setPosition(0);
    hoodEncoder.setPositionConversionFactor(1.0/12.0);

    hoodPID = hood.getPIDController();
    hoodPID.setP(4.0);

    turret.setSoftLimit(SoftLimitDirection.kForward, 100);
    turret.setSoftLimit(SoftLimitDirection.kReverse, -100);

    turretEncoder = turret.getEncoder();
    turretEncoder.setPositionConversionFactor(360.0/112.0);
    turretPID = turret.getPIDController();
    turretPID.setP(0.05);

    turretEncoder.setPosition(cancoder.getPosition());


    canifier.setLEDOutput(1, LEDChannel.LEDChannelC);
    canifier.setLEDOutput(0, LEDChannel.LEDChannelA);

  }
    public void setVelocity(double velocity) {
      //setpoint = velocity;
      shooter1.set(ControlMode.PercentOutput, velocity);
    }

    public void setPIDVelocity(double velocity) {
      shooter1.set(ControlMode.Velocity, rpmtonativeunits(velocity));
      setpoint = velocity;
    }

    public void setHood(double position){
      hoodPID.setReference(position, ControlType.kPosition);
    }

    public boolean isAtSpeed(){
      return true;
    }

    public double getVelocity(){
      return nativeunitstorpm(shooter1.getSelectedSensorVelocity());
    }

    public double rpmtonativeunits(double rpm){
      return (rpm/60.0/10.0*2048.0);
    }

    public double nativeunitstorpm(double nativeunits){
      return (nativeunits*60.0*10.0/2048.0);
    }

    public double getShooterVelocityForDistance(double distance){
       return speedMap.getInterpolated(new InterpolatingDouble(distance)).value;
    }

    public double getHoodPositionForDistance(double distance){
      return hoodMap.getInterpolated(new InterpolatingDouble(distance)).value;
   }

   public void setTurretPosition(double position){
     turretPID.setReference(position, ControlType.kPosition);
   }

   public double getAngle(){
     return turretEncoder.getPosition();
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("hoodEncoder", hoodEncoder.getPosition());
    SmartDashboard.putNumber("turretEncoder", turretEncoder.getPosition());
    SmartDashboard.putNumber("turretAbsolute", cancoder.getPosition());
    SmartDashboard.putNumber("shooterVelocity", nativeunitstorpm(shooter1.getSelectedSensorVelocity()));
  }
}
