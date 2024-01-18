// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {

  CANSparkMax intakeMotor;
  


  /** Creates a new Intake. */
  public Intake() {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeMotor = new CANSparkMax(Constants.Intake.motorPort, MotorType.kBrushless);
    
  
  }

 //Intake 1 sparkmax
//Feeder 2 neo 550
//Shooter 2
//Shows speed of motor(s) and sends to smart dashboard.
  @Override
  public void periodic() {
    SmartDashboard.putNumber("FrontIntakeSpeed", intakeMotor.get());
   
  }

  public void speed(double speed) {

  }
//
}
