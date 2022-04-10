// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
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
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

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
    hoodMap.put(new InterpolatingDouble(63.5), new InterpolatingDouble(0.1));
    hoodMap.put(new InterpolatingDouble(100.0), new InterpolatingDouble(0.4));
    hoodMap.put(new InterpolatingDouble(165.0), new InterpolatingDouble(0.7));
    hoodMap.put(new InterpolatingDouble(220.0), new InterpolatingDouble(0.85));
    hoodMap.put(new InterpolatingDouble(368.5), new InterpolatingDouble(1.0));
  }
  private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> speedMap = new InterpolatingTreeMap<>();
  static {
    speedMap.put(new InterpolatingDouble(67.8), new InterpolatingDouble(1900.0));
    speedMap.put(new InterpolatingDouble(100.0), new InterpolatingDouble(1900.0));
    speedMap.put(new InterpolatingDouble(165.0), new InterpolatingDouble(2200.0)); //2200
    speedMap.put(new InterpolatingDouble(220.0), new InterpolatingDouble(2450.0));
    speedMap.put(new InterpolatingDouble(368.0), new InterpolatingDouble(3000.0));
  }
  private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> distanceMap = new InterpolatingTreeMap<>();
  static {
    distanceMap.put(new InterpolatingDouble(2000.0), new InterpolatingDouble(62.0));
    distanceMap.put(new InterpolatingDouble(2500.0), new InterpolatingDouble(100.0));
    distanceMap.put(new InterpolatingDouble(2800.0), new InterpolatingDouble(150.0));
    distanceMap.put(new InterpolatingDouble(3000.0), new InterpolatingDouble(240.0));
    distanceMap.put(new InterpolatingDouble(3420.0), new InterpolatingDouble(407.0));
  }

  private double setpoint = 0; //gets used in setPIDVelocity, probably doesn't ned to be a global variable

  /** Creates a new Shooter. */
  public Shooter() {  
    
    shooter1.setInverted(true);
    shooter1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shooter1.config_kF(0, 0.054);
    //shooter1.config_kP(0, 0.005102);
    shooter1.config_kP(0, 0.1);
    shooter1.configPeakOutputReverse(0);
    shooter1.configPeakOutputForward(1);
    shooter1.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms);
    shooter1.configVoltageCompSaturation(11);
    shooter1.enableVoltageCompensation(true);
    for (StatusFrame statusFrame : StatusFrame.values()) {
      shooter1.setStatusFramePeriod(statusFrame, 20);
      shooter2.setStatusFramePeriod(statusFrame, 255);
    }

    shooter2.setInverted(TalonFXInvertType.OpposeMaster);
    shooter2.set(ControlMode.Follower, shooter1.getDeviceID());

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
    turretPID.setP(0.02);
    turretPID.setOutputRange(-.5, .5);

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
      shooter1.set(ControlMode.Velocity, rpmtonativeunits(velocity), DemandType.ArbitraryFeedForward, 0.05);
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

    //derek
    public double getDistanceFromShooterVelocity(double velocity){
      return distanceMap.getInterpolated(new InterpolatingDouble(velocity)).value;
   }

    public double getHoodPositionForDistance(double distance){
      return hoodMap.getInterpolated(new InterpolatingDouble(distance)).value;
   }

   //derek
   public void setTurretPosition(double position, double angular){
     double spinFactor = createSmartDashboardNumber("Spin On Spot Factor", 0.0);
     angular = 0;//angular * spinFactor; //rad/sec to percent output
     turretPID.setReference(position, ControlType.kPosition, 0, angular, ArbFFUnits.kPercentOut); 
     //turretPID.setReference(value, ctrl, pidSlot, arbFeedforward, arbFFUnits) //options are percent output or voltage
   }

   public void setTurretSpeed(double speed){
     turret.set(speed);
   }

   public double getAngle(){
     return turretEncoder.getPosition();
   }
   
   public boolean isReady(){
     return true;
   }

   //derek, taken from chief delphi
   public double createSmartDashboardNumber(String key, double defValue) {

    // See if already on dashboard, and if so, fetch current value
    double value = SmartDashboard.getNumber(key, defValue);
  
    // Make sure value is on dashboard, puts back current value if already set
    // otherwise puts back default value
    SmartDashboard.putNumber(key, value);
  
    return value;
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
