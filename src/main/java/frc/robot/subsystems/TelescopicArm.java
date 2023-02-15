// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopicArm extends SubsystemBase {
  XboxController controller = new XboxController(3);
  /** Creates a new TelescopicArm. */
  CANSparkMax pivotMotor = new CANSparkMax(Constants.Elevator.PIVOT_MOTORID, MotorType.kBrushless);
  PIDController winchPidController = new PIDController(0.01, 0, 0);
  PIDController pivotPidController;
  RelativeEncoder pivotEncoder;

  CANSparkMax winchMotor = new CANSparkMax(Constants.Elevator.EXTENDING_MOTORID, MotorType.kBrushless);
  RelativeEncoder winchEncoder;

  
  boolean atWinchSetpoint = false;
  boolean atPivotSetpoint = false;

  boolean isInverted = false;

  public enum winchPosition{
    CLOSED,
    LOW,
    MID,
    HIGH
  }

  public enum pivotPosition{
    LOW,
    MID,
    HIGH
  }

  winchPosition winchPos;
  pivotPosition pivotPos;
  
  public TelescopicArm() {
    winchPos = winchPosition.CLOSED;
    //Extend Motor Setup
    winchMotor.setInverted(false);
    winchMotor.restoreFactoryDefaults();
    winchMotor.setSmartCurrentLimit(60);
    winchMotor.setIdleMode(IdleMode.kBrake);
    winchEncoder = winchMotor.getEncoder();
    winchEncoder.setPosition(0);
    
    // //Pivot Motor Setup
    pivotMotor.setInverted(false);
    pivotMotor.restoreFactoryDefaults();
    pivotMotor.setSmartCurrentLimit(60);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotEncoder = pivotMotor.getEncoder();
    pivotEncoder.setPosition(0);

    // //Winch PID
    // winchPidController.setP(Constants.Elevator.WINCH_PID[0]);
    // winchPidController.setI(Constants.Elevator.WINCH_PID[1]);
    // winchPidController.setD(Constants.Elevator.WINCH_PID[2]);
    // winchPidController.setTolerance(10);

    // //PIVOT PID
    // pivotPidController.setP(Constants.Elevator.PIVOT_PID[0]);
    // pivotPidController.setI(Constants.Elevator.PIVOT_PID[1]);
    // pivotPidController.setD(Constants.Elevator.PIVOT_PID[2]);
    // pivotPidController.setTolerance(10);

  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("winchEncoder", getWinchEncoder());
    SmartDashboard.putNumber("pivotEncoder", getPivotEncoder());
    SmartDashboard.putNumber("winch Speed", winchEncoder.getVelocity());
    moveWinch(controller.getLeftY());
    movePivot(controller.getRightY());
    // This method will be called once per scheduler run
  }

  
  public void zeroWinch(){
    winchEncoder.setPosition(0);
  }
  
  public void zeroPivot(){
    pivotEncoder.setPosition(0);
  }
  
  //Functions for ease
  public void zeroTelescopicArm(){
    zeroWinch();
    zeroPivot();
  }
  public void moveWinch(double speed){
    if(controller.getLeftBumper()){
      winchMotor.set(speed);
    }
    else if(!controller.getLeftBumper()){
      if(winchEncoder.getPosition() > 281 || winchEncoder.getPosition() < 0){
      winchMotor.set(0);
    }
      else if(winchEncoder.getPosition() > 275){
      winchMotor.set(speed/2);
    }
      else{
      winchMotor.set(speed);
    }
  }
  }

  public void movePivot(double speed){
    pivotMotor.set(speed);
  }

  public double getWinchEncoder(){
    return winchEncoder.getPosition();
  }
  public double getPivotEncoder(){
    return 0;
    // return pivotEncoder.getPosition();
  }

}
