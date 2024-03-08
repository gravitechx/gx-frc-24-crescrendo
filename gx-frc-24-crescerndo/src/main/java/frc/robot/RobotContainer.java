// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import frc.robot.auto.modes.DepTwoGrabOne;
//import frc.robot.auto.modes.ScoreTwo;
//import frc.robot.auto.modes.StraightToDepo;
//import frc.robot.auto.modes.TestAuto;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Shooter shooter = new Shooter();
  private final Feeder feed = new Feeder();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final Limelight limelight = new Limelight(feed, intake);
  private final Lights lights = new Lights();

  private final Joystick controller = new Joystick(0);

  private final int translationAxis = 1; // 1
  private final int strafeAxis = 0; // 0
  private final int rotationAxis = 2; // 2
  
  private final JoystickButton zeroGyro = new JoystickButton(controller, 8);
  private final JoystickButton robotCentric = new JoystickButton(controller, 7);
  // private final JoystickButton halfSpeed = new JoystickButton(controller, 10);
  // private final JoystickButton quarterSpeed = new JoystickButton(controller, 11);
  // private final JoystickButton changeMode = new JoystickButton(controller, 12);
  // private final JoystickButton lock = new JoystickButton(controller, 13);
  private final JoystickButton intakeButton = new JoystickButton(controller, Constants.OI.intakeButton);
  private final JoystickButton feedButton = new JoystickButton(controller, Constants.OI.feedButton);
  private final JoystickButton shootButton = new JoystickButton(controller, Constants.OI.shootButton);
  private final JoystickButton shootSlowButton = new JoystickButton(controller, Constants.OI.shootSlowButton);
  private final JoystickButton rightClimberUpButton = new JoystickButton(controller, Constants.OI.leftClimberUpButton);
  private final JoystickButton rightClimberDownButton = new JoystickButton(controller, Constants.OI.leftClimberDownButton);
  private final JoystickButton leftClimberUpButton = new JoystickButton(controller, Constants.OI.rightClimberUpButton);
  private final JoystickButton leftClimberDownButton = new JoystickButton(controller, Constants.OI.rightClimberDownButton);
  private final JoystickButton bothClimberUpButton = new JoystickButton(controller, Constants.OI.bothClimberUpButton);
  private final JoystickButton bothClimberDownButton = new JoystickButton(controller, Constants.OI.bothClimberDownButton);

  private final JoystickButton increaseM1Button = new JoystickButton(controller, 10);
  private final JoystickButton decreaseM1Button = new JoystickButton(controller, 9);
  private final JoystickButton increaseM2Button = new JoystickButton(controller, 12);
  private final JoystickButton decreaseM2Button = new JoystickButton(controller, 11);
  //private final JoystickButton increaseTopSpeed = new JoystickButton(controller, 6);
  //private final JoystickButton decreaseTopSpeed = new JoystickButton(controller, 4);
  private final JoystickButton primeShooterButton = new JoystickButton(controller, 5);
  private final POVButton lockDirection = new POVButton(controller, Constants.OI.POVNorth);
  private final POVButton unlockDirection = new POVButton(controller, Constants.OI.POVSouth);

  // private final JoystickButton musicStartButton = new JoystickButton(controller, Constants.OI.musicStartButton);
  // private final JoystickButton musicStopButton = new JoystickButton(controller, Constants.OI.musicStopButton);

  private final Swerve s_Swerve = new Swerve();
  private final Mode modeControl = Mode.getInstance();

  private double motor1Speed = 1;
  private double motor2Speed = 1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> intake.spin(.7)));
    NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> intake.spin(0)));
    NamedCommands.registerCommand("Shoot", new InstantCommand(() -> shooter.spinOnly(.7, .7)));
    NamedCommands.registerCommand("StopShoot", new InstantCommand(() -> shooter.spinOnly(0, 0)));
    NamedCommands.registerCommand("Feed", new InstantCommand(() -> feed.spin(.7, intake)));
    NamedCommands.registerCommand("StopFeed", new InstantCommand(() -> feed.spin(0, intake)));
    NamedCommands.registerCommand("OnlyFeed", new InstantCommand(() -> feed.spinOnly(.7)));
    NamedCommands.registerCommand("StopOnlyFeed", new InstantCommand(() -> feed.spinOnly(0)));
    NamedCommands.registerCommand("Prep", new InstantCommand(() -> Constants.primeShooter = true));
    NamedCommands.registerCommand("StopPrep", new InstantCommand(() -> Constants.primeShooter = false));

    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve, 
        () -> -controller.getRawAxis(translationAxis),
        () -> -controller.getRawAxis(strafeAxis), 
        () -> -controller.getRawAxis(rotationAxis), 
        () -> robotCentric.getAsBoolean()
      )
    );

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    intakeButton.onTrue(new InstantCommand(() -> intake.spin(1)));
    intakeButton.onFalse(new InstantCommand(() -> intake.spin(0)));
    feedButton.onTrue(new InstantCommand(() -> feed.spin(.3, intake)));
    feedButton.onFalse(new InstantCommand(() -> feed.spin(0, intake)));
//     feedButton.onTrue(new InstantCommand(() -> limelight.showLimelight()));
    
// lockDirection.onTrue(new InstantCommand(() -> Constants.stopSide = 0));
// unlockDirection.onTrue(new InstantCommand(() -> Constants.stopSide = 1));


    // shootButton.onTrue(new InstantCommand(() -> shooter.spin((controller.getRawAxis(3) + 1) / 2, (controller.getRawAxis(3) + 1) / 2.5, feed, intake)));
    shootButton.onTrue(new InstantCommand(() -> shooter.spinOnly(motor1Speed, motor2Speed)));
    shootButton.onFalse(new InstantCommand(() -> shooter.spinOnly(0, 0)));
    // shootSlowButton.onTrue(new InstantCommand(() -> shooter.spinOnly(.27 / 2, .27)));
    // shootSlowButton.onFalse(new InstantCommand(() -> shooter.spinOnly(0, 0)));
    // candySetButton.onTrue(new InstantCommand(() -> ))
    // leftClimberUpButton.onTrue(new InstantCommand(() -> climber.spinLeft(0.5)));
    // leftClimberDownButton.onTrue(new InstantCommand(() -> climber.spinLeft(-0.5)));
    // rightClimberUpButton.onTrue(new InstantCommand(() -> climber.spinRight(0.5)));
    // rightClimberDownButton.onTrue(new InstantCommand(() -> climber.spinRight(-0.5)));
    // leftClimberUpButton.onFalse(new InstantCommand(() -> climber.spinLeft(0)));
    // leftClimberDownButton.onFalse(new InstantCommand(() -> climber.spinLeft(0)));
    // rightClimberUpButton.onFalse(new InstantCommand(() -> climber.spinRight(0)));
    // rightClimberDownButton.onFalse(new InstantCommand(() -> climber.spinRight(0)));

    // bothClimberUpButton.onTrue(new InstantCommand(() -> climber.spinRight(0.5)));
    // bothClimberUpButton.onTrue(new InstantCommand(() -> climber.spinLeft(0.5)));
    // bothClimberDownButton.onTrue(new InstantCommand(() -> climber.spinRight(-0.5)));
    // bothClimberDownButton.onTrue(new InstantCommand(() -> climber.spinLeft(-0.5)));
    // bothClimberUpButton.onFalse(new InstantCommand(() -> climber.spinRight(0.0)));
    // bothClimberUpButton.onFalse(new InstantCommand(() -> climber.spinLeft(0.0)));
    // bothClimberDownButton.onFalse(new InstantCommand(() -> climber.spinRight(0.0)));
    // bothClimberDownButton.onFalse(new InstantCommand(() -> climber.spinLeft(0.0)));

    // intakeButton.onTrue(new InstantCommand(() -> Lights.setColor(255, 0, 0)));
    feedButton.onFalse(new InstantCommand(() -> Lights.setColor(255, 0, 0)));
    // shootButton.onTrue(new InstantCommand(() -> Lights.setColor(0, 0, 255)));


    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    // halfSpeed.onTrue(new InstantCommand(() -> modeControl.changeSpeed(.5)));
    // halfSpeed.onFalse(new InstantCommand(() -> modeControl.changeSpeed(1)));
    // quarterSpeed.onTrue(new InstantCommand(() -> modeControl.changeSpeed(.25)));
    // quarterSpeed.onFalse(new InstantCommand(() -> modeControl.changeSpeed(1)));
    // changeMode.onTrue(new InstantCommand(() -> modeControl.changeMode()));

    increaseM1Button.onTrue(new InstantCommand(() -> motor1Speed += .01));
    increaseM1Button.onFalse(new InstantCommand(() -> SmartDashboard.putNumber("m1Speed", motor1Speed)));
    decreaseM1Button.onTrue(new InstantCommand(() -> motor1Speed -= .01));
    decreaseM1Button.onFalse(new InstantCommand(() -> SmartDashboard.putNumber("m1Speed", motor1Speed)));
    increaseM2Button.onTrue(new InstantCommand(() -> motor2Speed += .01));
    increaseM2Button.onFalse(new InstantCommand(() -> SmartDashboard.putNumber("m2Speed", motor2Speed)));
    decreaseM2Button.onTrue(new InstantCommand(() -> motor2Speed -= .01));
    decreaseM2Button.onFalse(new InstantCommand(() -> SmartDashboard.putNumber("m2Speed", motor2Speed)));

    // increaseTopSpeed.onTrue(new InstantCommand(() -> Constants.topSpeed+= 0.01));
    // increaseTopSpeed.onTrue(new InstantCommand(() -> SmartDashboard.putNumber("topSpeed", Constants.topSpeed)));
    // decreaseTopSpeed.onTrue(new InstantCommand(() -> Constants.topSpeed+= -0.01));
    // decreaseTopSpeed.onTrue(new InstantCommand(() -> SmartDashboard.putNumber("topSpeed", Constants.topSpeed)));
    primeShooterButton.onTrue(new InstantCommand(() -> Constants.primeShooter = (Constants.primeShooter) ? false : true));



    // musicStartButton.onTrue(new InstantCommand(() -> s_Swerve.music.play()));
    // musicStopButton.onTrue(new InstantCommand(() -> s_Swerve.music.stop()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return AutoBuilder.buildAuto("Blue4NoteE");
    // s_Swerve.resetOdometry(PathPlannerPath.fromChoreoTrajectory("NewPath").getPreviewStartingHolonomicPose());
    // return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("NewPath"));
  }
}

