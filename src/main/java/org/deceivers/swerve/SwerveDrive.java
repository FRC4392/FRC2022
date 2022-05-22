package org.deceivers.swerve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

public class SwerveDrive {
    private final SwerveModule[] mModules;
    private final int numModules;
    private final SwerveDriveKinematics mKinematics;
    private final SwerveDriveOdometry mSwerveDriveOdometry;
    private final HolonomicDriveController mDriveController;
    private final DoubleSupplier mGyroAngle;
    private ProfiledPIDController rotationPIDController = new ProfiledPIDController(10,.1,.1,new TrapezoidProfile.Constraints(8, 2));

    public SwerveDrive(DoubleSupplier gyroAngle, SwerveModule... modules){
        mGyroAngle = gyroAngle;
        numModules = modules.length;
        mModules = Arrays.copyOf(modules, numModules);

        Translation2d[] moduleLocations = new Translation2d[numModules];
        for (int i = 0; i < numModules; i++){
            moduleLocations[i] = mModules[i].getModuleLocation();
        }

        mKinematics = new SwerveDriveKinematics(moduleLocations);

        rotationPIDController.enableContinuousInput(-180.0, 180.0);

        mDriveController = new HolonomicDriveController(new PIDController(4,0,0), new PIDController(4,0,0), rotationPIDController);
        mSwerveDriveOdometry = new SwerveDriveOdometry(mKinematics, Rotation2d.fromDegrees(mGyroAngle.getAsDouble()));

        Arrays.stream(mModules).forEach(SwerveModule::init);
    }

	public void stop() {
        Arrays.stream(mModules).forEach(SwerveModule::stop);
	}

    public void drive(double forward, double strafe, double azimuth, boolean fieldRelative){
        ChassisSpeeds speeds;
        if (fieldRelative){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, azimuth, Rotation2d.fromDegrees(mGyroAngle.getAsDouble()));
        } else {
            speeds = new ChassisSpeeds(forward, strafe, azimuth);
        }
        SwerveModuleState[] states = mKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 1);
        for (int i = 0; i < numModules; i++){
            mModules[i].set(states[i]);
        }

    }

    public void driveClosedLoop(double forward, double strafe, double azimuth, boolean fieldRelative){
        ChassisSpeeds speeds;
        if (fieldRelative){
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, azimuth, Rotation2d.fromDegrees(mGyroAngle.getAsDouble()));
        } else {
            speeds = new ChassisSpeeds(forward, strafe, azimuth);
        }
        SwerveModuleState[] states = mKinematics.toSwerveModuleStates(speeds);
        //SwerveDriveKinematics.normalizeWheelSpeeds(states, 1);
        for (int i = 0; i < numModules; i++){
            mModules[i].setClosedLoop(states[i]);
        }
    }

    public Pose2d getPosition(){
        return mSwerveDriveOdometry.getPoseMeters();
    }

    public Pose2d updateOdometry(){
        SwerveModuleState[] states = new SwerveModuleState[numModules];
        for (int i = 0; i < numModules; i++) {
            states[i] = mModules[i].getState();
        }
                
        return mSwerveDriveOdometry.update(Rotation2d.fromDegrees(mGyroAngle.getAsDouble()), states);
    }

    public ChassisSpeeds getChassisSpeeds(){
        SwerveModuleState[] states = new SwerveModuleState[numModules];
        for (int i = 0; i < numModules; i++) {
            states[i] = mModules[i].getState();
        }
        ChassisSpeeds chassisSpeeds = mKinematics.toChassisSpeeds(states);
        return(chassisSpeeds);
    }
    
    public void followPath(double startTime, PathPlannerTrajectory pptrajectory){
        PathPlannerState goal = (PathPlannerState) pptrajectory.sample(Timer.getFPGATimestamp() - startTime);
        

        ChassisSpeeds speeds = mDriveController.calculate(getPosition(), goal, Rotation2d.fromDegrees(goal.holonomicRotation.getDegrees()));

        driveClosedLoop(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
        SmartDashboard.putNumber("goalx", goal.poseMeters.getX());
        SmartDashboard.putNumber("xerror", goal.poseMeters.getX() - getPosition().getX());
        SmartDashboard.putNumber("goaly", goal.poseMeters.getY());
        SmartDashboard.putNumber("yerror", goal.poseMeters.getY() - getPosition().getY());
        SmartDashboard.putNumber("goalrot", goal.holonomicRotation.getDegrees());
        SmartDashboard.putNumber("roterror", goal.holonomicRotation.getDegrees() - getPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("xVel", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("yVel", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("rotationPIDsetpoint", rotationPIDController.getSetpoint().position);
        SmartDashboard.putNumber("rotationPIDgoal", rotationPIDController.getGoal().position);
        SmartDashboard.putNumber("rotationPIDerror", rotationPIDController.getPositionError());
    }

    public void log(){
        Arrays.stream(mModules).forEach(SwerveModule::log);
        Pose2d pose = getPosition();
        SmartDashboard.putNumber("SwerveXLocation", pose.getX());
        SmartDashboard.putNumber("SwerveYLocation", pose.getY());
        SmartDashboard.putNumber("SwerveRotation", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("GyroAngle", mGyroAngle.getAsDouble());
        SmartDashboard.putNumber("poseRotation", pose.getRotation().getDegrees());
    }

    public void setLocation(double x, double y, double angle){
        Pose2d newPose = new Pose2d(x, y, Rotation2d.fromDegrees(angle));
        mSwerveDriveOdometry.resetPosition(newPose, Rotation2d.fromDegrees(angle));
    }

    public void setModulesAngle(double angle, int module){
        mModules[module].setAngle(angle);
    }


}