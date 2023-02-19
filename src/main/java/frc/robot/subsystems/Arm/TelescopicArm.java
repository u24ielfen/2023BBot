// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopicArm extends SubsystemBase {
  Encoder pivotEncoder  = new Encoder(1, 2);

  XboxController controller = new XboxController(3);
  boolean inverted = false;
  /** Creates a new TelescopicArm. */
  public static TelescopicArm instance;
  CANSparkMax pivotMotor = new CANSparkMax(Constants.Elevator.PIVOT_MOTORID, MotorType.kBrushless);
  PIDController winchPidController = new PIDController(0.01, 0, 0);
  PIDController pivotPidController;

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
    SmartDashboard.putBoolean("Zero Arm", false);
    SmartDashboard.putBoolean("Arm Manual Control", false);
    winchPos = winchPosition.CLOSED;
    //Extend Motor Setup
    winchMotor.setInverted(false);
    winchMotor.restoreFactoryDefaults();
    winchMotor.setSmartCurrentLimit(40);
    winchMotor.setIdleMode(IdleMode.kBrake);
    winchEncoder = winchMotor.getEncoder();
    winchEncoder.setPosition(0);
    
    // //Pivot Motor Setup
    pivotMotor.setInverted(false);
    pivotMotor.restoreFactoryDefaults();
    pivotMotor.setSmartCurrentLimit(40);
    pivotMotor.setIdleMode(IdleMode.kBrake);
  }
  
  @Override
  public void periodic() {
    if(SmartDashboard.getBoolean("Zero Arm", true)){
      SmartDashboard.putBoolean("Zero Arm", false);
      zeroTelescopicArm();
    }
    SmartDashboard.putNumber("winchEncoder", getWinchEncoder());
    SmartDashboard.putNumber("pivotEncoder", getPivotEncoder());
    SmartDashboard.putNumber("winch Speed", winchEncoder.getVelocity());

      moveWinch(controller.getLeftY());
      movePivot(controller.getRightY());
  }

  
  public void zeroWinch(){
    winchEncoder.setPosition(0);
  }
  
  public void zeroPivot(){
    pivotEncoder.reset();
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
      if((winchEncoder.getPosition() > 281 && speed > 0) || (winchEncoder.getPosition() < 0 && speed < 0)){
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
    if(controller.getLeftBumper()){
      pivotMotor.set(speed);
    }
    else if(!controller.getLeftBumper()){
      if(pivotEncoder.get() > 281 || pivotEncoder.get() < 0){
      pivotMotor.set(0);
    }
      else if(pivotEncoder.get() > 275){
      pivotMotor.set(speed/2);
    }
      else{
      pivotMotor.set(speed);
    }
  }  }


  public void winchToBam(double ticks){
    if(getWinchEncoder() > ticks - 5 && getWinchEncoder() < ticks + 5){
      moveWinch(0);
    }
    else if(getWinchEncoder() < ticks - 20){
      moveWinch(0.8);
    }
    else if(getWinchEncoder() > ticks - 20 && getWinchEncoder() < ticks){
      moveWinch(0.3);
    }
    else if(getWinchEncoder() > ticks){
      moveWinch(-0.8);
    }
  }
  

  public void pivotToBam(double ticks){
    if(getPivotEncoder() > ticks - 5 && getPivotEncoder() < ticks + 5){
      moveWinch(0);
    }
    if(getPivotEncoder() < ticks - 5){
      movePivot(-0.4);
    }
    else if(getPivotEncoder() > ticks + 5){
      movePivot(0.4);
    }
  }

  public double getWinchEncoder(){
    return winchEncoder.getPosition();
  }
  public double getPivotEncoder(){
    return pivotEncoder.get();
  }

public static TelescopicArm getInstance() {
  if(instance == null){

    return new TelescopicArm();
  } 
  return instance; 
}

public void stopPivot(){
  pivotMotor.set(0);
}

public void stopWinch(){
  winchMotor.set(0);
}

public void toggleInverted(){
  if(inverted){
    inverted = false;
  }
  else{ 
    inverted = true;
  }
}
}
