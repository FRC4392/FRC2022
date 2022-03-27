package frc.robot.swerve;

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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

public class SwerveDrive {
    private final SwerveModule[] mModules;
    private final int numModules;
    private final SwerveDriveKinematics mKinematics;
    private final SwerveDriveOdometry mSwerveDriveOdometry;
    private final HolonomicDriveController mDriveController;
    private final DoubleSupplier mGyroAngle;
    private Trajectory trajectory = new Trajectory();

    //move gyro out of this class?


    public SwerveDrive(DoubleSupplier gyroAngle, SwerveModule... modules){
        mGyroAngle = gyroAngle;
        numModules = modules.length;
        mModules = Arrays.copyOf(modules, numModules);

        Translation2d[] moduleLocations = new Translation2d[numModules];
        for (int i = 0; i < numModules; i++){
            moduleLocations[i] = mModules[i].getModuleLocation();
        }

        mKinematics = new SwerveDriveKinematics(moduleLocations);

        mDriveController = new HolonomicDriveController(new PIDController(4,0,0), new PIDController(4,0,0), new ProfiledPIDController(2,.1,.1,new TrapezoidProfile.Constraints(6.28, 3.14)));
        mSwerveDriveOdometry = new SwerveDriveOdometry(mKinematics, Rotation2d.fromDegrees(mGyroAngle.getAsDouble()));

        Arrays.stream(mModules).forEach(SwerveModule::init);
        Arrays.stream(mModules).forEach(SwerveModule::init);

//        TrajectoryConfig config = new TrajectoryConfig(2.5, 1);
//        config.setKinematics(mKinematics);
//        //config.setReversed(true);
//        config.addConstraint(new SwerveDriveKinematicsConstraint(mKinematics, 3));
//        List<Translation2d> midpoints = new ArrayList<>();
//
//        midpoints.add(new Translation2d(2.0, 2.0));
//        midpoints.add(new Translation2d(4.0, -2.0));
//        midpoints.add(new Translation2d(6.0, 0.0));
        
        //trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(), midpoints, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180)), config);

        // try {
        //     trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("output/Bounce.wpilib.json"));
        // } catch (Exception e){
        //     DriverStation.reportError("Unable to open trajectory: " + "Slalom.wpilib.json", e.getStackTrace());

        // }

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

    public void followPath(double startTime){
        Trajectory.State goal = trajectory.sample(Timer.getFPGATimestamp() - startTime);

        ChassisSpeeds speeds = mDriveController.calculate(getPosition(), goal, new Rotation2d());

        driveClosedLoop(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond, false);
        SmartDashboard.putNumber("goalx", goal.poseMeters.getX());
        SmartDashboard.putNumber("xerror", goal.poseMeters.getX() - getPosition().getX());
        SmartDashboard.putNumber("goaly", goal.poseMeters.getY());
        SmartDashboard.putNumber("yerror", goal.poseMeters.getY() - getPosition().getY());
        SmartDashboard.putNumber("goalrot", goal.poseMeters.getRotation().getDegrees());
        SmartDashboard.putNumber("xVel", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("yVel", speeds.vyMetersPerSecond);
    }

    public void log(){
        Arrays.stream(mModules).forEach(SwerveModule::log);
        Pose2d pose = getPosition();
        SmartDashboard.putNumber("SwerveXLocation", pose.getX());
        SmartDashboard.putNumber("SwerveYLocation", pose.getY());
        SmartDashboard.putNumber("SwerveRotation", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("GyroAngle", mGyroAngle.getAsDouble());
    }

    public void setLocation(double x, double y, double angle){
        Pose2d newPose = new Pose2d(x, y, Rotation2d.fromDegrees(angle));
        mSwerveDriveOdometry.resetPosition(newPose, Rotation2d.fromDegrees(angle));
    }

    public void setStartPostion(){
        //mSwerveDriveOdometry.resetPosition(trajectory.getInitialPose(), Rotation2d.fromDegrees(0));
    }

    public void setModulesAngle(double angle){
        for (SwerveModule swerveModule : mModules) {
            swerveModule.setAngle(angle);
        }
    }


}