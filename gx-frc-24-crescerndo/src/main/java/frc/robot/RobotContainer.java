// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

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
import edu.wpi.first.wpilibj.ADXL345_I2C.Axes;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
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
  // Instantiating our robot subsystem
  private final Shooter shooter = new Shooter();
  private final Feeder feed = new Feeder();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final Limelight limelight = new Limelight(feed, intake);
  private final Lights lights = new Lights();

  private final Joystick controller = new Joystick(Constants.OI.controllerPort);
  private final Joystick buttons = new Joystick(Constants.OI.buttonPanelPort);

  private final JoystickButton testNormalButton = new JoystickButton(buttons, Constants.OI.testNormalButton);
  private final JoystickButton testReverseButton = new JoystickButton(buttons, Constants.OI.testReverseButton);
  private final JoystickButton rightClimberUpButtonPanel = new JoystickButton(buttons, Constants.OI.rightClimberUpButtonPanel);
  private final JoystickButton rightClimberDownButtonPanel = new JoystickButton(buttons, Constants.OI.rightClimberDownButtonPanel);
 private final JoystickButton leftClimberUpButtonPanel = new JoystickButton(buttons, Constants.OI.leftClimberUpButtonPanel);
  private final JoystickButton leftClimberDownButtonPanel = new JoystickButton(buttons, Constants.OI.leftClimberDownButtonPanel);

  // Justin's key binds
 
  private final JoystickButton zeroGyro = new JoystickButton(controller, Constants.OI.resetGyro);
 
  private final JoystickButton intakeButton = new JoystickButton(controller, Constants.OI.intakeButton);
  private final JoystickButton feedButton = new JoystickButton(controller, Constants.OI.feedButton);
 private final JoystickButton shootButton = new JoystickButton(controller, Constants.OI.shootButton);
 private final JoystickButton shootSlowButton = new JoystickButton(controller, Constants.OI.shootSlowButton);
  private final JoystickButton rightClimberUpButton = new JoystickButton(controller, Constants.OI.leftClimberUpButton);
  private final JoystickButton rightClimberDownButton = new JoystickButton(controller, Constants.OI.leftClimberDownButton);
  private final JoystickButton leftClimberUpButton = new JoystickButton(controller, Constants.OI.rightClimberUpButton);
  private final JoystickButton leftClimberDownButton = new JoystickButton(controller, Constants.OI.rightClimberDownButton);
 

  private  final  JoystickButton increaseM1Button = new JoystickButton(buttons, 7);
  private final JoystickButton decreaseM1Button = new JoystickButton(buttons, 8);
  private final JoystickButton increaseM2Button = new JoystickButton(buttons, 9);
  private final JoystickButton decreaseM2Button = new JoystickButton(buttons, 10);
  private final JoystickButton shootOverRobotButton= new JoystickButton(controller, 5);
  
  private final JoystickButton reverseFeedButton = new JoystickButton(controller, Constants.OI.reverseFeed);
  
  
  private final Trigger trigger = new Trigger(() -> controller.getRawAxis(3) == 1);
  private final Trigger trigger2 = new Trigger(() -> controller.getRawAxis(3) == -1); //DO NOT TOUCH
  
 

//Justin's Unused key binds

  //private final JoystickButton decreaseshootOverRobotButton= new JoystickButton(controller, 6);
  //private final JoystickButton increaseTopSpeed = new JoystickButton(controller, 6);
  //private final JoystickButton decreaseTopSpeed = new JoystickButton(controller, 4);
  // private final JoystickButton primeShooterButton = new JoystickButton(controller, Constants.OI.primeShooterButton);

   // private final JoystickButton bothClimberUpButton = new JoystickButton(controller, Constants.OI.bothClimberUpButton);
  // private final JoystickButton bothClimberDownButton = new JoystickButton(controller, Constants.OI.bothClimberDownButton);

   // private final JoystickButton halfSpeed = new JoystickButton(controller, 10);
  // private final JoystickButton quarterSpeed = new JoystickButton(controller, 11);
  // private final JoystickButton changeMode = new JoystickButton(controller, 12);
  // private final JoystickButton lock = new JoystickButton(controller, 13);

   // End of justin's key binds
  
  //Ronan's key binds
  

//   private final POVButton zeroGyro = new POVButton(controller, Constants.OI.POVNorth);
//   private final POVButton reverseFeedButton = new POVButton(controller, Constants.OI.POVSouth);
//  private final JoystickButton intakeButton = new JoystickButton(controller, Constants.OI.intakeButton);
//   private final JoystickButton feedButton = new JoystickButton(controller, Constants.OI.feedButton);
//   private final JoystickButton rightClimberUpButton = new JoystickButton(controller, Constants.OI.leftClimberUpButton);
//   private final JoystickButton rightClimberDownButton = new JoystickButton(controller, Constants.OI.leftClimberDownButton);
//   private final JoystickButton leftClimberUpButton = new JoystickButton(controller, Constants.OI.rightClimberUpButton);
//   private final JoystickButton leftClimberDownButton = new JoystickButton(controller, Constants.OI.rightClimberDownButton);
//   private final Boolean dumbDumb = Constants.OI.dummy;
//    private final Trigger longShootButton = new Trigger(() -> controller.getRawAxis(2) >= 0.25);
//   private final Trigger shootButton = new Trigger(() -> controller.getRawAxis(3) >= 0.25);

//  private final Trigger shootButton = new Trigger(getTriggerPressed());
// private final Trigger shootOverRobotButton = new Trigger(getTriggerPressed());




 // private final POVButton sourceIntakeButton = new POVButton(controller, Constants.OI.POVEast);
//private final Joystick shootButton = controller.getRawAxis(Constants.OI.longShotAxis);

  //End of Ronan's key binds
 
  // private final JoystickButton musicStartButton = new JoystickButton(controller, Constants.OI.musicStartButton);
  // private final JoystickButton musicStopButton = new JoystickButton(controller, Constants.OI.musicStopButton);

 


  // Swerve stuff
  private final Swerve s_Swerve = new Swerve();
  private final Mode modeControl = Mode.getInstance();

  private double motor1Speed = 1;
  private double motor2Speed = 1;
  // ^^ Shooter motor variables that are changed on the fly

  public RobotContainer() {
    configureAutoCommands();

    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve, 
        () -> -controller.getRawAxis(Constants.OI.translationAxis),
        () -> -controller.getRawAxis(Constants.OI.strafeAxis),
        () -> -controller.getRawAxis(Constants.OI.rotationAxis),
        () -> false
      )
    );

    configureBindings();
  }

  private void configureCommands() {
    // intakeButton = new JoystickButton(controller, Constants.OI.intakeButton);
    // feedButton = new JoystickButton(controller, Constants.OI.feedButton);
    // shootButton = new JoystickButton(controller, Constants.OI.shootButton);
  }

  private void configureAutoCommands() {
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> intake.spin(.7)));
    NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> intake.spin(0)));
    NamedCommands.registerCommand("Shoot", new InstantCommand(() -> shooter.spinOnly(.9, .9)));
    NamedCommands.registerCommand("StopShoot", new InstantCommand(() -> shooter.spinOnly(0, 0)));
    NamedCommands.registerCommand("Feed", new InstantCommand(() -> feed.spin(.7, intake)));
    NamedCommands.registerCommand("StopFeed", new InstantCommand(() -> feed.spin(0, intake)));
    NamedCommands.registerCommand("OnlyFeed", new InstantCommand(() -> feed.spinOnly(.7)));
    NamedCommands.registerCommand("StopOnlyFeed", new InstantCommand(() -> feed.spinOnly(0)));
    NamedCommands.registerCommand("Prep", new InstantCommand(() -> Constants.primeShooter = true));
    NamedCommands.registerCommand("StopPrep", new InstantCommand(() -> Constants.primeShooter = false));
    NamedCommands.registerCommand("FlipGyro", new InstantCommand(() -> s_Swerve.flipZeroGyro()));
  }

  private void configureBindings() {
//Justin

// shootSlowButton.onTrue(new InstantCommand(() -> shooter.spinOnly(.08, .35)));
//  shootSlowButton.onFalse(new InstantCommand(() -> shooter.spinOnly(.08, .35)));
 
//Ronan


//Both
// shootButton.onTrue(new InstantCommand(() -> shooter.spinOnly(1, 1)));
 shootButton.onFalse(new InstantCommand(() -> shooter.spinOnly(.08, .35))); // speed:.06, speed3: 0.31

 shootOverRobotButton.onTrue(new InstantCommand(() ->shooter.spinOnly(.98,.32)));
shootOverRobotButton.onFalse(new InstantCommand(() ->shooter.spinOnly(0.06,0.31)));


reverseFeedButton.onTrue(new InstantCommand(() -> feed.spin(-.5, intake)));
    reverseFeedButton.onFalse(new InstantCommand(() -> feed.spin(0, intake)));
    reverseFeedButton.onTrue(new InstantCommand(() -> shooter.spinOnly(-.5, -.5)));
    reverseFeedButton.onFalse(new InstantCommand(() -> shooter.spinOnly(.08, .35)));

    intakeButton.onTrue(new InstantCommand(() -> intake.spin(1)));
    intakeButton.onFalse(new InstantCommand(() -> intake.spin(0)));
    feedButton.onTrue(new InstantCommand(() -> feed.spin(.3, intake)));
    feedButton.onFalse(new InstantCommand(() -> feed.spin(0, intake)));
    // feedButton.onTrue(new InstantCommand(() -> limelight.showLimelight()));
    
    // lockDirection.onTrue(new InstantCommand(() -> Constants.stopSide = 0));
    // unlockDirection.onTrue(new InstantCommand(() -> Constants.stopSide = 1));

    //shootButton.onTrue(new InstantCommand(() -> shooter.spinOnly((controller.getRawAxis(Constants.OI.longShotButton)), (controller.getRawAxis(Constants.OI.longShotButton)))));
    //shootSlowButton.onTrue(new InstantCommand(() -> shooter.spinOnly((controller.getRawAxis(Constants.OI.shotButton)*0.06), (controller.getRawAxis(Constants.OI.shotButton)*0.31))));

    // Change with buttons, joystick 9 and 11 are down, 10 and 12 are up. 9-10 and 11-12.
    shootButton.onTrue(new InstantCommand(() -> shooter.spinOnly(motor1Speed, motor2Speed)));

   // stopShootButton.onTrue(new InstantCommand(() -> shooter.spinOnly(0, 0)));
   
   
    // leftClimberUpButton.onTrue(new InstantCommand(() -> climber.spinLeft(1)));
    // leftClimberDownButton.onTrue(new InstantCommand(() -> climber.spinLeft(-1)));
    // rightClimberUpButton.onTrue(new InstantCommand(() -> climber.spinRight(1)));
    // rightClimberDownButton.onTrue(new InstantCommand(() -> climber.spinRight(-1)));
    // leftClimberUpButton.onFalse(new InstantCommand(() -> climber.spinLeft(0)));
    // leftClimberDownButton.onFalse(new InstantCommand(() -> climber.spinLeft(0)));
    // rightClimberUpButton.onFalse(new InstantCommand(() -> climber.spinRight(0)));
    // rightClimberDownButton.onFalse(new InstantCommand(() -> climber.spinRight(0)));
    
    // rightClimberUpButton.onTrue(new InstantCommand(() -> intake.spin(1)));
    // rightClimberUpButton.onTrue(new InstantCommand(() -> feed.spinOnly(.1)));
    // rightClimberUpButton.onFalse(new InstantCommand(() -> feed.spin(0, intake)));

    // Not usable rn, for both climbers
    // bothClimberUpButton.onTrue(new InstantCommand(() -> climber.spinRight(0.5)));
    // bothClimberUpButton.onTrue(new InstantCommand(() -> climber.spinLeft(0.5)));
    // bothClimberDownButton.onTrue(new InstantCommand(() -> climber.spinRight(-0.5)));
    // bothClimberDownButton.onTrue(new InstantCommand(() -> climber.spinLeft(-0.5)));
    // bothClimberUpButton.onFalse(new InstantCommand(() -> climber.spinRight(0.0)));
    // bothClimberUpButton.onFalse(new InstantCommand(() -> climber.spinLeft(0.0)));
    // bothClimberDownButton.onFalse(new InstantCommand(() -> climber.spinRight(0.0)));
    // bothClimberDownButton.onFalse(new InstantCommand(() -> climber.spinLeft(0.0)));

    trigger.onTrue(new InstantCommand(() -> Lights.setColor(255, 0, 0)));
    trigger.onFalse(new InstantCommand(() -> Lights.setColor(0, 0, 255)));
    trigger2.onTrue(new InstantCommand(() -> Lights.setColor(0, 255, 0)));
    trigger2.onFalse(new InstantCommand(() -> Lights.setColor(0, 0, 255)));

    // intakeButton.onTrue(new InstantCommand(() -> Lights.setColor(255, 0, 0)));
    feedButton.onFalse(new InstantCommand(() -> Lights.setColor(255, 0, 0)));
    // shootButton.onTrue(new InstantCommand(() -> Lights.setColor(0, 0, 255)));

    // Comment out for testing amp on the fly
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    // halfSpeed.onTrue(new InstantCommand(() -> modeControl.changeSpeed(.5)));
    // halfSpeed.onFalse(new InstantCommand(() -> modeControl.changeSpeed(1)));
    // quarterSpeed.onTrue(new InstantCommand(() -> modeControl.changeSpeed(.25)));
    // quarterSpeed.onFalse(new InstantCommand(() -> modeControl.changeSpeed(1)));
    // changeMode.onTrue(new InstantCommand(() -> modeControl.changeMode()));

    // Comment in for testing amp on the fly. 
    increaseM1Button.onTrue(new InstantCommand(() -> motor1Speed += .01));
    increaseM1Button.onFalse(new InstantCommand(() -> SmartDashboard.putNumber("m1Speed", motor1Speed)));
    decreaseM1Button.onTrue(new InstantCommand(() -> motor1Speed -= .01));
    decreaseM1Button.onFalse(new InstantCommand(() -> SmartDashboard.putNumber("m1Speed", motor1Speed)));
    increaseM2Button.onTrue(new InstantCommand(() -> motor2Speed += .01));
    increaseM2Button.onFalse(new InstantCommand(() -> SmartDashboard.putNumber("m2Speed", motor2Speed)));
    decreaseM2Button.onTrue(new InstantCommand(() -> motor2Speed -= .01));
    decreaseM2Button.onFalse(new InstantCommand(() -> SmartDashboard.putNumber("m2Speed", motor2Speed)));

    // Coment out for testing amp on the fly.



   
    // reverseFeedButton.onTrue(new InstantCommand(() -> Constants.topSpeed = (Constants.topSpeed == 1) ? .7 : 1));

    // increaseTopSpeed.onTrue(new InstantCommand(() -> Constants.topSpeed+= 0.01));
    // increaseTopSpeed.onTrue(new InstantCommand(() -> SmartDashboard.putNumber("topSpeed", Constants.topSpeed)));
    // decreaseTopSpeed.onTrue(new InstantCommand(() -> Constants.topSpeed+= -0.01));
    // decreaseTopSpeed.onTrue(new InstantCommand(() -> SmartDashboard.putNumber("topSpeed", Constants.topSpeed)));
    // primeShooterButton.onTrue(new InstantCommand(() -> Constants.primeShooter = (Constants.primeShooter) ? false : true));

    // musicStartButton.onTrue(new InstantCommand(() -> s_Swerve.music.play()));
    // musicStopButton.onTrue(new InstantCommand(() -> s_Swerve.music.stop()));

    testNormalButton.onTrue(new InstantCommand(() -> shooter.spin(1, 1, feed, intake)));
    testNormalButton.onFalse(new InstantCommand(() -> shooter.spin(0, 0, feed, intake)));
    testReverseButton.onTrue(new InstantCommand(() -> shooter.spinOnly(-1, -1)));
    testReverseButton.onTrue(new InstantCommand(() -> feed.spin(-1, intake)));
    testReverseButton.onFalse(new InstantCommand(() -> shooter.spin(0, 0, feed, intake)));
    rightClimberUpButtonPanel.onTrue(new InstantCommand(() -> climber.spinRight(-1)));
    rightClimberDownButtonPanel.onTrue(new InstantCommand(() -> climber.spinRight(1)));
    leftClimberUpButtonPanel.onTrue(new InstantCommand(() -> climber.spinLeft(1)));
    leftClimberDownButtonPanel.onTrue(new InstantCommand(() -> climber.spinLeft(-1)));
    rightClimberUpButtonPanel.onFalse(new InstantCommand(() -> climber.spinRight(0)));
    rightClimberDownButtonPanel.onFalse(new InstantCommand(() -> climber.spinRight(0)));
    leftClimberUpButtonPanel.onFalse(new InstantCommand(() -> climber.spinLeft(0)));
    leftClimberDownButtonPanel.onFalse(new InstantCommand(() -> climber.spinLeft(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return AutoBuilder.buildAuto("NewSupport");
    // s_Swerve.resetOdometry(PathPlannerPath.fromChoreoTrajectory("NewPath").getPreviewStartingHolonomicPose());
    // return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("NewPath"));
  }
}

