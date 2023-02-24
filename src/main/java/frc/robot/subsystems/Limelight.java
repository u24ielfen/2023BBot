// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightConstants;
import frc.robot.LimelightConstants.ledMode;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  NetworkTable table;
  NetworkTableEntry targetY;
  NetworkTableEntry targetX;
  NetworkTableEntry targetArea;
  NetworkTableEntry area;

  DoubleSubscriber targetYAgain;


  public Limelight() {
      table = NetworkTableInstance.getDefault().getTable("limelight");
      targetY = table.getEntry("ty");
      targetX = table.getEntry("tx");
      area = table.getEntry("ta");
      targetYAgain = table.getDoubleTopic("ty").subscribe(0.0);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Cone Distance", getDistance(0.0));
    table.getEntry("ledMode").setNumber(3);
    SmartDashboard.putNumber("Double Topic", targetYAgain.getAsDouble());
    setLedMode(ledMode.FORCE_ON);
  }


  public double getDistance(double targetHeight){
      //Get the vertical angle offset from limelight
      double verticalOffsetAngle = targetY.getDouble(0.0);

      //get the angle to goal in degrees
      double angleToGoalDegrees = LimelightConstants.limelightMountAngle + verticalOffsetAngle;

      //convert angle to radians
      double angleToGoalRadians = angleToGoalDegrees * (3.1415926/180);

      //calculate distance to target
      double dist = (targetHeight-LimelightConstants.limelightMountHeight)/Math.tan(angleToGoalRadians);
      return dist;
  }

  public double getAngleX(){
      return targetX.getDouble(0.0);
  }

  //Allows user to swap between on, off, and blinking LEDS
  public void setLedMode(LimelightConstants.ledMode mode){
    table.getEntry("ledMode").setNumber(3);
  }

  //Toggle between vision processing and driver cam
  public void setCamMode(int mode){
    table.getEntry("camMode").setNumber(mode);
  }

  //Toggle between different stream layouts
  public void setStreamMode(int mode){
    table.getEntry("stream").setNumber(mode);
  }
  public boolean hasTarget(){
    return area.getDouble(0.0) > 0;
  }

  //Take snapshot and reset snapshot mode
  public void snapshot(int snapMode){
    table.getEntry("snapshot").setNumber(snapMode);
  }

  //Change pipelines
  public void setPipeline(int pipeline){
    table.getEntry("pipeline").setNumber(pipeline);
  }
}
