// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopicArm extends SubsystemBase {
  DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);
  XboxController controller = new XboxController(3);
  boolean inverted = false;
  /** Creates a new TelescopicArm. */
  public static TelescopicArm instance;
  CANSparkMax pivotMotor = new CANSparkMax(Constants.Elevator.PIVOT_MOTORID, MotorType.kBrushless);
  CANSparkMax pivotMotor_slave = new CANSparkMax(24, MotorType.kBrushless);
  PIDController winchPidController = new PIDController(0.01, 0, 0);
  PIDController pivotPidController;
  
  CANSparkMax winchMotor = new CANSparkMax(Constants.Elevator.EXTENDING_MOTORID, MotorType.kBrushless);
  
  RelativeEncoder winchEncoder = winchMotor.getEncoder();
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
    winchMotor.setSmartCurrentLimit(40);
    winchMotor.setIdleMode(IdleMode.kBrake);
    // winchEncoder.setInverted(true);
    
    // //Pivot Motor Setup
    pivotMotor_slave.setInverted(false);
    pivotMotor_slave.restoreFactoryDefaults();
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setInverted(false);
    pivotMotor.restoreFactoryDefaults();
    pivotMotor.setSmartCurrentLimit(40);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotEncoder.setPositionOffset(0.16);
    SmartDashboard.putBoolean("No Limit", false);

  }
  
  @Override
  public void periodic() {
    if(SmartDashboard.getBoolean("Zero Pivot", true)){
      SmartDashboard.putBoolean("Zero Pivot", false);
      zeroPivot();
    }
  if(SmartDashboard.getBoolean("Zero Arm", true)){
    SmartDashboard.putBoolean("Zero Arm", false);
    zeroWinch();
  }

    SmartDashboard.putNumber("winchEncoder", getWinchEncoder());
    SmartDashboard.putNumber("pivotEncoder", getPivotEncoder());
    SmartDashboard.putNumber("ELEVATOR ANGLE", getArmAngle());
    
      // moveWinch(-controller.getLeftY());
      moveWinch(controller.getLeftY());
      movePivot(controller.getRightY());
  }

  
  public void zeroWinch(){
    winchEncoder.setPosition(0);
  }
  
  public void zeroPivot(){
    pivotEncoder.reset();
  }
  
  public void zeroTelescopicArm(){
    zeroWinch();
    // zeroPivot();
  }
  public void moveWinch(double speed){
    SmartDashboard.putNumber("Winch Speed", speed);
    winchMotor.set(speed);
  }

  public void movePivot(double speed){
    if(Math.abs(speed) <= 0.1){
      speed = 0;
    }

      pivotMotor.set(speed);
      pivotMotor_slave.set(speed);
  } 


  public void winchToBam(double ticks, double maxSpeed){
    SmartDashboard.putBoolean("Winch TO", true);
   if(getWinchEncoder() < ticks - 0.2){
      moveWinch(maxSpeed);
    }
    else if(getWinchEncoder() > ticks - 0.2 && getWinchEncoder() < ticks){
      moveWinch(0.2);
    }
    else if(getWinchEncoder() > ticks + 0.2){
      moveWinch(-maxSpeed);
    }
    
    else if(getWinchEncoder() < ticks + 0.2 && getWinchEncoder() > ticks){
      moveWinch(-0.2);
    }
  }
  

  public void pivotToBam(double ticks, double maxSpeed){
    if(getPivotEncoder() < ticks - 0.008){
      movePivot(-maxSpeed);
    }
    else if(getPivotEncoder() > ticks - 0.008 && getPivotEncoder() < ticks){
      movePivot(-0.1);
    }
    
    else if(getPivotEncoder() > ticks + 0.008){
      movePivot(maxSpeed);
    }
    else if(getPivotEncoder() < ticks + 0.008){
      movePivot(0.1);
    }
  }

  public double getWinchEncoder(){
    return winchEncoder.getPosition();
  }
  public double getPivotEncoder(){
    return pivotEncoder.getAbsolutePosition();
  }

public static TelescopicArm getInstance() {
  if(instance == null){

    return new TelescopicArm();
  } 
  return instance; 
}

public void movePivot180(){
  if(getPivotEncoder() < 0.2 && getPivotEncoder() > 0.4){
    movePivot(0.2);
  }
  
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

public double getArmAngle(){
  double angle = (getPivotEncoder() - Constants.Elevator.startingPose_Pivot)/(2* Math.PI);
  return angle;
}
}
