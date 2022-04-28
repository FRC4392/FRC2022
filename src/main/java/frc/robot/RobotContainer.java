// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
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
    autoChooser.addOption("5 Ball", Double.valueOf(1));
    autoChooser.setDefaultOption("2 Ball", Double.valueOf(2));
    autoChooser.addOption("Do Nothing", Double.valueOf(-1));
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
    POVButton hint45BackLeftButton = new POVButton(operatorController, 225);
    POVButton hint45BackRightButton = new POVButton(operatorController, 135);
    POVButton hintBackButton = new POVButton(operatorController, 180);


    // Trigger ClimbTrigger = new Trigger(() -> {
    //   return (operatorController.getLeftTriggerAxis() > 0) && (operatorController.getRightTriggerAxis()>0);
    // });

    JoystickButton ClimbTrigger = new JoystickButton(operatorController, XboxController.Button.kLeftStick.value);

    //trigger for reverse tower Left trigger (done)
    Trigger reverseTowerTrigger = new Trigger(() -> {
      return (operatorController.getLeftTriggerAxis() > 0) && !(operatorController.getRightTriggerAxis() > 0);
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
      return (operatorController.getRightTriggerAxis() > 0) && !(operatorController.getLeftTriggerAxis() > 0);
    });
    
    //for robot relative
    hintForwardButton.whileHeld(new HintTurretDirection(shooter, 0, driveTrain));
    hint45LeftButton.whileHeld(new HintTurretDirection(shooter, -45, driveTrain));
    hint45RightButton.whileHeld(new HintTurretDirection(shooter, 45, driveTrain));
    hintLeftButton.whileHeld(new HintTurretDirection(shooter, -90, driveTrain));
    hintRightButton.whileHeld(new HintTurretDirection(shooter, 90, driveTrain));
    hint45BackLeftButton.whileHeld(new HintTurretDirection(shooter, -135, driveTrain));
    hint45RightButton.whileHeld(new HintTurretDirection(shooter, 135, driveTrain));
    hintBackButton.whileHeld(new HintTurretDirection(shooter, 180, driveTrain));



/*
    //for field relative (maybe)
    hintForwardButton.whenPressed(new HintTurretDirection(shooter, 0, driveTrain));
    hint45LeftButton.whenPressed(new HintTurretDirection(shooter, -45, driveTrain));
    hint45RightButton.whenPressed(new HintTurretDirection(shooter, 45, driveTrain));
    hintLeftButton.whenPressed(new HintTurretDirection(shooter, -90, driveTrain));
    hintRightButton.whenPressed(new HintTurretDirection(shooter, 90, driveTrain));
*/

    ClimbTrigger.whileHeld(new ClimbCommand(climber, shooter, operatorController));

    reverseTowerTrigger.whileActiveContinuous(new ReverseTowerCommand(sequencer));
    manualMoveButton.whenPressed(new ManualMoveTurretCommand(shooter, operatorController));

    fixedShotOneButton.whileHeld(new FixedShotCommand(shooter, 2200, .5));
    fixedShotTwoButton.whileHeld(new FixedShotCommand(shooter, 400, 1));
    fixedShotThreeButton.whileHeld(new FixedShotCommand(shooter, 300, 1));
    fixedShotFourButton.whileHeld(new FixedShotCommand(shooter, 6000, 1));
    
    brakeButton.whileActiveContinuous(new SwerveBrakeCommand(driveTrain));
    autoEjectTrigger.whileActiveContinuous(new AutoEjectCommand(sequencer, shooter));


    //JoystickButton climbButton = new JoystickButton(driverController, 7);
    //JoystickButton reverseClimbButton = new JoystickButton(driverController, 8);
    JoystickButton shootButton = new JoystickButton(operatorController, 6);
    //POVButton upClimb = new POVButton(driverController, 0);
    //POVButton downClimb = new POVButton(driverController, 180);
    JoystickButton feedButton = new JoystickButton(operatorController, 5);
 

    intakeButton.toggleWhenActive(new IntakeCommand(intake, conveyor, true));

    shootButton.whileHeld(new AutoShootCommand(shooter, limelight, driveTrain));

    feedButton.whileHeld(new FeedWhenReadyCommand(sequencer, shooter, conveyor));

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
    if (autoChooser.getSelected() == 1){
    return new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new AutoShootCommand(shooter, limelight, driveTrain).withTimeout(2),
        new AutoFeedShooter(sequencer, conveyor)
        ),
      new ParallelDeadlineGroup(
        new FollowPathPlannerPath(PathPlanner.loadPath("New Path", 3, 1), true, driveTrain), 
        new IntakeCommand(intake, conveyor, false)
        ),
      new ParallelDeadlineGroup(
        new AutoShootCommand(shooter, limelight, driveTrain).withTimeout(2),
        new AutoFeedShooter(sequencer, conveyor)
        ),
      new ParallelDeadlineGroup(
        new FollowPathPlannerPath(PathPlanner.loadPath("New New Path", 3, 1), false, driveTrain),
        new IntakeCommand(intake, conveyor, false)
        ),
      new ParallelCommandGroup(
        new IntakeCommand(intake, conveyor, true),
        new AutoShootCommand(shooter, limelight, driveTrain),
        new AutoFeedShooter(sequencer, conveyor)
        )
    );
    }else if (autoChooser.getSelected() == 2){
      return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
        new FollowPathPlannerPath(PathPlanner.loadPath("New New New Path", 2, 1), true, driveTrain), 
        new IntakeCommand(intake, conveyor, false)
        ), 
        new ParallelCommandGroup(
          new IntakeCommand2(intake, true),
          new AutoShootCommand(shooter, limelight, driveTrain),
          new AutoFeedShooter(sequencer, conveyor)
        )
      );
    } else {
      return new WaitCommand(1);
    }
    // double startPosition = autoChooser.getSelected().doubleValue();
    // ParallelCommandGroup shootandfeed = new ParallelCommandGroup(new AutoShootCommand(shooter, limelight, driverController, driveTrain), new AutoFeedCommand(sequencer));
    // return new SequentialCommandGroup(new DumbAutoDrive(driveTrain, startPosition, intake), shootandfeed);
  }
}
