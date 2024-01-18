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


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

  CANSparkMax leftFeederMotor;
  CANSparkMax rightFeederMotor;

  /** Creates a new Feeder. */
  public Feeder() {
    leftFeederMotor = new CANSparkMax(Constants.Feeder.leftMotorPort, MotorType.kBrushless);
    rightFeederMotor = new CANSparkMax(Constants.Feeder.rightMotorPort, MotorType.kBrushless);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("FrontFeederSpeed", leftFeederMotor.get());
    SmartDashboard.putNumber("FrontFeederSpeed", rightFeederMotor.get());




  } 
  public void speed(double speed) {
    rightFeederMotor.set(-speed);
    leftFeederMotor.set(speed);
  }
}
