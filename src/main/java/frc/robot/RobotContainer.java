// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.HintTurretDirection;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetTurretPositionCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.indexCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotCore;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight.LedMode;

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
  Limelight limelight = new Limelight();

  //OI
  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    limelight.setLEDMode(LedMode.kOn);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton intakeButton = new JoystickButton(driverController, 5);

    POVButton hintForwardButton = new POVButton(operatorController, 0);
    POVButton hint45LeftButton = new POVButton(operatorController, 315);
    POVButton hint45RightButton = new POVButton(operatorController, 45);
    POVButton hintLeftButton = new POVButton(operatorController, 270);
    POVButton hintRightButton = new POVButton(operatorController, 90);

    Trigger ClimbTrigger = new Trigger(() -> {
      return (operatorController.getLeftTriggerAxis() > 0) && (operatorController.getRightTriggerAxis()>0);
    });

    hintForwardButton.whileHeld(new HintTurretDirection(shooter, 0));
    hint45LeftButton.whileHeld(new HintTurretDirection(shooter, -45));
    hint45RightButton.whileHeld(new HintTurretDirection(shooter, 45));
    hintLeftButton.whileHeld(new HintTurretDirection(shooter, -90));
    hintRightButton.whileHeld(new HintTurretDirection(shooter, 90));

    ClimbTrigger.whileActiveContinuous(new ClimbCommand(climber, shooter, operatorController));


    //JoystickButton climbButton = new JoystickButton(driverController, 7);
    //JoystickButton reverseClimbButton = new JoystickButton(driverController, 8);
    JoystickButton shootButton = new JoystickButton(operatorController, 6);
    //POVButton upClimb = new POVButton(driverController, 0);
    //POVButton downClimb = new POVButton(driverController, 180);
    JoystickButton feedButton = new JoystickButton(operatorController, 5);
 

    intakeButton.whileHeld(new IntakeCommand(intake, conveyor));

    shootButton.whileHeld(new AutoShootCommand(shooter, limelight));

    feedButton.whileHeld(new FeedShooter(sequencer));

    driveTrain.setDefaultCommand(new DriveCommand(driveTrain, driverController));
    sequencer.setDefaultCommand(new indexCommand(sequencer));

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
