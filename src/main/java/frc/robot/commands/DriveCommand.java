/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveCommand extends CommandBase {
  public final Drivetrain mDrivetrain;
  public XboxController mController;
  private boolean lastScan;

  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(5);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(0);

  final double TARGET_AREA_PERCENT = 15;

  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(4);

  // Change this to match the name of your camera (comes from the UI thing,
  // gloworm is default with this image)
  PhotonCamera camera = new PhotonCamera("gloworm");

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.08;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.01;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  public DriveCommand(Drivetrain Drivetrain, XboxController XboxController) {

    mDrivetrain = Drivetrain;
    mController = XboxController;

    addRequirements(mDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double forwardSpeed;
    double rotationSpeed;
   
    if (mController.getAButton()) {
      // Vision-alignment mode
      // Query the latest result from PhotonVision
      var result = camera.getLatestResult();

      if (result.hasTargets()) {

        // First calculate range

        // Method 1: Height in camera FOV vs height difference (works better the greater
        // difference in height from camera to target)
        // does not work with 0 height difference
        // double range = PhotonUtils.calculateDistanceToTargetMeters(
        // CAMERA_HEIGHT_METERS,
        // TARGET_HEIGHT_METERS,
        // CAMERA_PITCH_RADIANS,
        // Units.degreesToRadians(result.getBestTarget().getPitch()));

        // -1.0 required to ensure positive PID controller effort _increases_ range
        // forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS); //For
        // the height difference range method

        // Area based range estimation, kinda bad as the percieved size of the vest will
        // change depending if the person is running away
        // or not
        double range = result.getBestTarget().getArea();
        forwardSpeed = -forwardController.calculate(range, TARGET_AREA_PERCENT); 

        System.out.println(Double.toString(range));

        // Also calculate angular power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
      } else {
        // If we have no targets, stay still.
        forwardSpeed = 0;
        rotationSpeed = 0;
      }
    } else {
      // Manual Driver Mode
      forwardSpeed = -mController.getLeftY();
      rotationSpeed = mController.getRightX();
    }

    // Movement deadzone
    if (Math.abs(forwardSpeed) < 0.09) {
      forwardSpeed = 0;
    }

    if (Math.abs(rotationSpeed) < 0.09) {
      rotationSpeed = 0;
    }

    if (mController.getRawButton(7) & !lastScan) {
      mDrivetrain.resetGyro();
    }
    lastScan = mController.getRawButton(7);

    // boolean fieldRelative = !mController.getRightBumper();
    mDrivetrain.drive(-.1*forwardSpeed, 0.0, .5*rotationSpeed, false);

    // mDrivetrain.setModulesAngle(xVel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
