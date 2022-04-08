package org.deceivers.swerve;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleV3 implements SwerveModule {

    private final CANSparkMax mAzimuthMotor;
    private final CANSparkMax mDriveMotor;
    private final RelativeEncoder mAzimuthEncoder;
    private final RelativeEncoder mDriveEncoder;
    private final SparkMaxPIDController mDrivePID;
    private final SparkMaxPIDController mAzimuthPID;
    private final Translation2d mLocation;
    private final String mName; 
    private boolean isInverted;
    private double setpoint;

    //need to update the speed to m/s

    public SwerveModuleV3(CANSparkMax azimuthMotor, CANSparkMax driveMotor,
            Translation2d location, String name) {
        mDriveMotor = driveMotor;
        mAzimuthMotor = azimuthMotor;
        mLocation = location;
        mName = name;
        mDriveMotor.restoreFactoryDefaults();
        mAzimuthMotor.restoreFactoryDefaults();

        mAzimuthEncoder = mAzimuthMotor.getEncoder();
        mDriveEncoder = mDriveMotor.getEncoder();
        mAzimuthMotor.setInverted(true);
        mDriveEncoder.setPositionConversionFactor(.004356);
        mDriveEncoder.setVelocityConversionFactor((0.029/42.0));
        mDriveEncoder.setPosition(0);
        mDrivePID = mDriveMotor.getPIDController();
        mDrivePID.setFF(0.30);
        mDrivePID.setP(0.5);
        
        mAzimuthPID = mAzimuthMotor.getPIDController();
    }

    // Sets the drive motor speed in open loop mode
    public void setSpeed(double speed) {
        mDriveMotor.set(speed);
    }

    // Sets the rotation speed of the azimuth motor in open loop mode
    public void setRotation(double rotation) {
        mAzimuthMotor.set(rotation);
    }

    // Gets the speed of the drive motor
    public double getSpeed() {
        return mDriveEncoder.getVelocity();
    }

    // Gets the rotation position of the azimuth module
    public double getRotation() {
        return mAzimuthEncoder.getPosition();
    }

    @Override
    public Translation2d getModuleLocation() {
        return mLocation;
    }

    //@Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getRotation()));
    }

    @Override
    public void init() {
        mAzimuthMotor.getEncoder().setPositionConversionFactor(360.0/35.94);
        mAzimuthMotor.setIdleMode(IdleMode.kBrake);
        mAzimuthMotor.setSmartCurrentLimit(20);
        mAzimuthPID.setP(0.05);

        mDriveMotor.setInverted(true);
        mDriveMotor.setClosedLoopRampRate(0);
        mDriveMotor.setIdleMode(IdleMode.kCoast);
        mDriveMotor.setSmartCurrentLimit(40, 60, 5700);

        mDriveMotor.burnFlash();
        mAzimuthMotor.burnFlash();

        setAzimuthZero();
    }

    @Override
    public void log() {
        SmartDashboard.putNumber(mName + " Incremental Position", mAzimuthEncoder.getPosition());
        SmartDashboard.putNumber(mName + " Velocity", mDriveEncoder.getVelocity());
        SmartDashboard.putNumber(mName + "Drive Encoder Position", mDriveEncoder.getPosition());
        SmartDashboard.putNumber(mName + " Rotation Setpoint", setpoint);
        SmartDashboard.putNumber(mName + "Percent Output", mDriveMotor.get());
    }

    @Override
    public void set(SwerveModuleState drive) {

        // if (Math.abs(mAzimuthEncoder.getPosition() - mAzimuthAbsoluteEncoder.getPosition()) > 5){
        //     setAzimuthZero();
        // }

        double Angle = drive.angle.getDegrees();
        double Velocity = drive.speedMetersPerSecond;

        double azimuthPosition = mAzimuthEncoder.getPosition();
        double azimuthError = Math.IEEEremainder(Angle - azimuthPosition, 360);

        isInverted = Math.abs(azimuthError) > 90;
        if (isInverted) {
            azimuthError -= Math.copySign(180, azimuthError);
            Velocity = -Velocity;
        }
        setpoint = azimuthError + azimuthPosition;
        mAzimuthPID.setReference(setpoint, ControlType.kPosition);
        mDriveMotor.set(Velocity);
    }

    @Override
    public void setClosedLoop(SwerveModuleState drive){
        // if (Math.abs(mAzimuthEncoder.getPosition() - mAzimuthAbsoluteEncoder.getPosition()) > 1){
        //     //setAzimuthZero();
        // }

        double Angle = drive.angle.getDegrees();
        double Velocity = drive.speedMetersPerSecond;

        double azimuthPosition = mAzimuthEncoder.getPosition();
        double azimuthError = Math.IEEEremainder(Angle - azimuthPosition, 360);

        isInverted = Math.abs(azimuthError) > 90;
        if (isInverted) {
            azimuthError -= Math.copySign(180, azimuthError);
            Velocity = -Velocity;
        }
        setpoint = azimuthError + azimuthPosition;
        mAzimuthPID.setReference(setpoint, ControlType.kPosition);
        mDrivePID.setReference(Velocity, ControlType.kVelocity);
    }

    @Override
    public void stop(){
        mAzimuthMotor.set(0);
        mDriveMotor.set(0);
    }
    
    public void setAzimuthZero() {
        //calculate position to increments
        // double position = ((1-(mAnalog.getVoltage() / 3.3))*360)-180;

        // @SuppressWarnings("unused")
        // REVLibError err = mAzimuthEncoder.setPosition(position);
    }

	@Override
	public void setAngle(double angle) {
        mAzimuthPID.setReference(angle, ControlType.kPosition);
		
	}
    
}
