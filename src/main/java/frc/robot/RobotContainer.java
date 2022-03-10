// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotCore;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsytems
  Drivetrain driveTrain = new Drivetrain();
  Climber climber = new Climber();
  Intake intake = new Intake();
  Sequencer sequencer = new Sequencer();
  Shooter shooter = new Shooter();
  RobotCore pneumatics = new RobotCore();
  Conveyor conveyor = new Conveyor();

  //OI
  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton intakeButton = new JoystickButton(driverController, 6);
    intakeButton.whileHeld(new IntakeCommand(intake, conveyor, sequencer, shooter));

    driveTrain.setDefaultCommand(new DriveCommand(driveTrain, driverController));

    // JoystickButton dropIntake = new JoystickButton(driverController, 1);

    // dropIntake.whenPressed(null);

    // JoystickButton shootButton = new JoystickButton(operatorController, 1);

    // shootButton.whileHeld(null);

    // JoystickButton climbButton = new JoystickButton(operatorController, 1);

    // climbButton.whenPressed(null);

    // JoystickButton runSequencerButton = new JoystickButton(operatorController, 1);

    // runSequencerButton.whileHeld(null);

    // JoystickButton pointLeftButton = new JoystickButton(operatorController, 1);

    // pointLeftButton.whenPressed(null);

    // JoystickButton pointRightButton = new JoystickButton(operatorController, 1);

    // pointRightButton.whenPressed(null);

    // JoystickButton pointForwardButton = new JoystickButton(operatorController, 1);

    // pointForwardButton.whenPressed(null);

    // JoystickButton pointBackButton = new JoystickButton(operatorController, 1);

    // pointBackButton.whenPressed(null);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
