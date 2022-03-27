// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DumbAutoDrive;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.FeedWhenReadyCommand;
import frc.robot.commands.FixedShotCommand;
import frc.robot.commands.HintTurretDirection;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualMoveTurretCommand;
import frc.robot.commands.ReverseTowerCommand;
import frc.robot.commands.SwerveBrakeCommand;
import frc.robot.commands.AutoEjectCommand;
import frc.robot.commands.AutoFeedCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DefaultConveyor;
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
  Climber climber = new Climber();
  Intake intake = new Intake();
  Sequencer sequencer = new Sequencer();
  Shooter shooter = new Shooter();
  RobotCore pneumatics = new RobotCore();
  Conveyor conveyor = new Conveyor();
  Limelight limelight = new Limelight();
  SendableChooser<Double> autoChooser = new SendableChooser<>();

  //OI
  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1);

  Drivetrain driveTrain = new Drivetrain();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    limelight.setLEDMode(LedMode.kOff);
    autoChooser.addOption("Left", Double.valueOf(135));
    autoChooser.setDefaultOption("Right", Double.valueOf(-156));
    SmartDashboard.putData(autoChooser);
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

    //trigger for reverse tower Left trigger (done)
    Trigger reverseTowerTrigger = new Trigger(() -> {
      return (operatorController.getLeftTriggerAxis() > 0) &! (operatorController.getRightTriggerAxis() > 0);
    });

    //manual move turret Right Toggle down (done)
    JoystickButton manualMoveButton = new JoystickButton(operatorController, XboxController.Button.kRightStick.value);
    //fixed shot buttons A B X Y (done)
    JoystickButton fixedShotOneButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
    JoystickButton fixedShotTwoButton = new JoystickButton(operatorController, XboxController.Button.kB.value);
    JoystickButton fixedShotThreeButton = new JoystickButton(operatorController, XboxController.Button.kX.value);
    JoystickButton fixedShotFourButton = new JoystickButton(operatorController, XboxController.Button.kY.value);

    //swerve brake B button (done)
    JoystickButton brakeButton = new JoystickButton(driverController, XboxController.Button.kB.value);
    //auto feed overrides normal feed (done)

    //auto eject button Right trigger (done)
    Trigger autoEjectTrigger = new Trigger(() -> {
      return (operatorController.getRightTriggerAxis() > 0) &! (operatorController.getLeftTriggerAxis() > 0);
    });


    hintForwardButton.whileHeld(new HintTurretDirection(shooter, 0));
    hint45LeftButton.whileHeld(new HintTurretDirection(shooter, -45));
    hint45RightButton.whileHeld(new HintTurretDirection(shooter, 45));
    hintLeftButton.whileHeld(new HintTurretDirection(shooter, -90));
    hintRightButton.whileHeld(new HintTurretDirection(shooter, 90));

    ClimbTrigger.whileActiveContinuous(new ClimbCommand(climber, shooter, operatorController));

    reverseTowerTrigger.whileActiveContinuous(new ReverseTowerCommand(sequencer));
    manualMoveButton.whenPressed(new ManualMoveTurretCommand(shooter, operatorController));
    fixedShotOneButton.whenPressed(new FixedShotCommand(shooter, 500));
    fixedShotTwoButton.whenPressed(new FixedShotCommand(shooter, 500));
    fixedShotThreeButton.whenPressed(new FixedShotCommand(shooter, 500));
    fixedShotFourButton.whenPressed(new FixedShotCommand(shooter, 500));
    brakeButton.whileActiveContinuous(new SwerveBrakeCommand(driveTrain));
    autoEjectTrigger.whileActiveContinuous(new AutoEjectCommand(sequencer, shooter));


    //JoystickButton climbButton = new JoystickButton(driverController, 7);
    //JoystickButton reverseClimbButton = new JoystickButton(driverController, 8);
    JoystickButton shootButton = new JoystickButton(operatorController, 6);
    //POVButton upClimb = new POVButton(driverController, 0);
    //POVButton downClimb = new POVButton(driverController, 180);
    JoystickButton feedButton = new JoystickButton(operatorController, 5);
 

    intakeButton.toggleWhenActive(new IntakeCommand(intake, conveyor));

    shootButton.whileHeld(new AutoShootCommand(shooter, limelight));

    feedButton.whileHeld(new FeedWhenReadyCommand(sequencer, shooter));

    driveTrain.setDefaultCommand(new DriveCommand(driveTrain, driverController));
    sequencer.setDefaultCommand(new indexCommand(sequencer));
    conveyor.setDefaultCommand(new DefaultConveyor(conveyor));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    double startPosition = autoChooser.getSelected().doubleValue();
    ParallelCommandGroup shootandfeed = new ParallelCommandGroup(new AutoShootCommand(shooter, limelight), new AutoFeedCommand(sequencer));
    return new SequentialCommandGroup(new DumbAutoDrive(driveTrain, startPosition, intake), shootandfeed);
  }
}
