// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  CANSparkMax motor;
  DigitalInput sensor;
  DigitalInput sensor1;

  public Intake() {
    motor = new CANSparkMax(Constants.Intake.motorPort, MotorType.kBrushless);
    sensor = new DigitalInput(Constants.Intake.sensorPort);
    sensor1 = new DigitalInput(Constants.Intake.sensorPort1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("SensorOutput", sensor.get());
    SmartDashboard.putBoolean("Sensor1Output", sensor1.get());
    
    if (!sensor.get() || !sensor1.get()) {
      Lights.setColor(0, 255, 61);
    }
  }

  public void spin(double speed) {
    motor.set(-speed);
  }
}
