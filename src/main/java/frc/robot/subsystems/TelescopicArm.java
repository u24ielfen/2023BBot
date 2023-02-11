// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopicArm extends SubsystemBase {
  /** Creates a new TelescopicArm. */
  CANSparkMax pivotMotor = new CANSparkMax(Constants.Elevator.PIVOT_MOTORID, MotorType.kBrushless);
  RelativeEncoder pivotEncoder;

  CANSparkMax winchMotor = new CANSparkMax(Constants.Elevator.EXTENDING_MOTORID, MotorType.kBrushless);
  RelativeEncoder winchEncoder;

  PIDController winchPidController;
  PIDController pivotPidController;
  
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
    
    //Pivot Motor Setup
    pivotMotor.setInverted(false);
    pivotMotor.restoreFactoryDefaults();
    pivotMotor.setSmartCurrentLimit(60);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotEncoder = pivotMotor.getEncoder();
    pivotEncoder.setPosition(0);

    //Winch PID
    winchPidController.setP(Constants.Elevator.WINCH_PID[0]);
    winchPidController.setI(Constants.Elevator.WINCH_PID[1]);
    winchPidController.setD(Constants.Elevator.WINCH_PID[2]);
    winchPidController.setTolerance(10);

    //PIVOT PID
    pivotPidController.setP(Constants.Elevator.PIVOT_PID[0]);
    pivotPidController.setI(Constants.Elevator.PIVOT_PID[1]);
    pivotPidController.setD(Constants.Elevator.PIVOT_PID[2]);
    pivotPidController.setTolerance(10);

  }
  
  @Override
  public void periodic() {
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
  
  public void closeWinch(){
    setWinchPosition(0);
  }

  public void setWinchLow(){
    setWinchPosition(Constants.Elevator.WINCH_TICKS_TO_BOTTOM);
  }

  public void setWinchMid(){
    setWinchPosition(Constants.Elevator.WINCH_TICKS_TO_MID);
  }
  public void setWinchHigh(){
    setWinchPosition(Constants.Elevator.WINCH_TICKS_TO_TOP);
  }

  public void setPivotLow(){
    setPivotPosition(Constants.Elevator.PIVOT_TICKS_TO_BOTTOM);
  }

  public void setPivotMid(){
    setPivotPosition(Constants.Elevator.PIVOT_TICKS_TO_MID);
  }
  
  public void setPivotHigh(){
    setPivotPosition(Constants.Elevator.PIVOT_TICKS_TO_TOP);
  }

  public void setWinchPosition(double setPoint){
    winchPidController.setSetpoint(setPoint);
    double speed = winchPidController.calculate(winchEncoder.getPosition());
    winchMotor.set(speed);
    if(winchPidController.atSetpoint()){
      atWinchSetpoint = true;
      winchMotor.set(0);
    }
    else atWinchSetpoint = false;
    
    if(setPoint == Constants.Elevator.WINCH_TICKS_TO_BOTTOM){
      winchPos = winchPosition.LOW;
    }
    else if(setPoint == Constants.Elevator.WINCH_TICKS_TO_MID){
      winchPos = winchPosition.MID;
    }
    else if(setPoint == Constants.Elevator.WINCH_TICKS_TO_TOP){
      winchPos = winchPosition.HIGH;
    }
    else if(setPoint == 0){
      winchPos = winchPosition.CLOSED;
    }
  }

  public void setPivotPosition(double setPoint){
    pivotPidController.setSetpoint(setPoint);
    double speed = pivotPidController.calculate(pivotEncoder.getPosition());
    pivotMotor.set(speed);
    if(pivotPidController.atSetpoint()){
      atPivotSetpoint = true;
      pivotMotor.set(0);
    }
    
    if(setPoint == Constants.Elevator.PIVOT_TICKS_TO_BOTTOM){
      pivotPos = pivotPosition.LOW;
    }
    else if(setPoint == Constants.Elevator.PIVOT_TICKS_TO_MID){
      pivotPos = pivotPosition.MID;
    }
    else if(setPoint == Constants.Elevator.PIVOT_TICKS_TO_TOP){
      pivotPos = pivotPosition.HIGH;
    }
    
  }
  
  // public void turnWinch180Degrees(){
  //   pivotPidController.setSetpoint(pivotEncoder.getPosition() + 180);
  //   double speed = pivotPidController.calculate(pivotEncoder.getPosition());
  //   pivotMotor.set(speed);
  //   if(pivotPidController.atSetpoint()){
  //     atPivotSetpoint = true;
  //     pivotMotor.set(0);
  //   }
  //   else atPivotSetpoint = false;
  // }  
}
