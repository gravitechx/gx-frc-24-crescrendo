// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry tz;
  NetworkTableEntry ta;

  double x;
  double y;
  double z;
  double area;
  double[] limelightTable = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);  

  Feeder feeder;
  Intake intake;
  boolean a = false;
  
  /** Creates a new Limelight. */
  public Limelight(Feeder feeder, Intake intake) {
    this.feeder = feeder;
    this.intake = intake;

    // tx = table.getEntry("tx");
    // ty = table.getEntry("ty");
    // tz = table.getEntry("tz");
    // ta = table.getEntry("ta");
 
  }

  @Override
  public void periodic() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);  

    // This method will be called once per scheduler run

    //read values periodically
    // x = tx.getDouble(0.0);
    // y = ty.getDouble(0.0);
    // z = tz.getDouble(0.0);
    // area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("Limelight0", limelightTable[0]);
    SmartDashboard.putNumber("Limelight1", limelightTable[1]);
    SmartDashboard.putNumber("Limelight2", limelightTable[2]);
    SmartDashboard.putNumber("Limelight3", limelightTable[3]);
    SmartDashboard.putNumber("Limelight4", limelightTable[4]);
    SmartDashboard.putNumber("Limelight5", limelightTable[5]);

    if (Math.abs(limelightTable[0]) < .6 && Math.abs(limelightTable[0]) != 0) {
      if (Constants.primeShooter) {
        a = true;
        feeder.spinOnly(.7);
      }
    } else {
      if (a && Constants.primeShooter) {
        a = false;
        feeder.spinOnly(0);
      }
    }
  }

  public void showLimelight() {
    

    SmartDashboard.putNumber("limelightSide", limelightTable[0]);
    SmartDashboard.putNumber("limelightDistance", limelightTable[2]);


  }
}
