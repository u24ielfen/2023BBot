// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  DutyCycleEncoder intakeEncoder = new DutyCycleEncoder(1);
  CANSparkMax spinMotor = new CANSparkMax(Constants.Intake.MOTORID, MotorType.kBrushless);
  CANSparkMax chinMotor = new CANSparkMax(Constants.Intake.CHINID, MotorType.kBrushless);
  RelativeEncoder spinEncoder = spinMotor.getEncoder();
  RelativeEncoder chinEncoder = chinMotor.getEncoder();
  
  DigitalInput limitSwitch = new DigitalInput(2); 
  DigitalInput limitSwitch2 = new DigitalInput(5); 
  
  XboxController controller = new XboxController(3);
  static Intake instance;
  
  PIDController intakePID = new PIDController(Constants.Intake.PID[0], Constants.Intake.PID[1], Constants.Intake.PID[2]);
  public enum state{
    OPEN,
    CLOSED
  }
  public state currentState;
  public Intake() {
    SmartDashboard.putBoolean("Zero Chin", false);
    spinMotor.restoreFactoryDefaults();
    spinMotor.setInverted(true);
    spinMotor.setIdleMode(IdleMode.kBrake);
    chinMotor.restoreFactoryDefaults();
    chinMotor.setInverted(false);
    chinMotor.setIdleMode(IdleMode.kBrake);

  }
  public boolean getLimitSwitch(){
    return limitSwitch.get() || limitSwitch2.get();
  }
  @Override
  public void periodic() {
    if(SmartDashboard.getBoolean("Zero Chin", true)){
      SmartDashboard.putBoolean("Zero Chin", false);
      chinEncoder.setPosition(0);
    }
    if(controller.getXButton()){
      chinTicksToBam(2.33);
    }
    SmartDashboard.putNumber("Chin Position", chinEncoder.getPosition());
    SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
    SmartDashboard.putNumber("Encoder Up Top", intakeEncoder.getAbsolutePosition());
    if(controller.getRightBumper()){
      spinMotor.set(-0.8);
    } else if(controller.getLeftBumper() && limitSwitch.get() == false){
      spinMotor.set(0.8);
    } else{
      spinMotor.set(0);
    }

    
    if(controller.getAButton()){
      chinMotor.set(-0.5);
    } else if(controller.getBButton()){
      chinMotor.set(0.5);
    } else{
      chinMotor.set(0);
    }
    // This method will be called once per scheduler run
  }
  
  public state getState(){
    return currentState;
  } 

  public void stopSpinMotor(){
    spinMotor.set(0);
  }
  public void stopChinMotor(){
    chinMotor.set(0);
  }
  
  public void chinTicksToBam(double ticks){
    if(ticks - getChinEncoder() > 3){
      moveChinMotor(0.5);
    }
    else if(ticks - getChinEncoder() > 0.2 && ticks - getChinEncoder() < 3){
      moveChinMotor(0.1);
    }
    else if(ticks - getChinEncoder() < -3){
      moveChinMotor(-0.5);
    }
    else if(ticks - getChinEncoder() < -0.2 && ticks - getChinEncoder() > -3){
      moveChinMotor(-0.1);
    }
  }

  public void moveSpinMotor(double speed){
      spinMotor.set(speed);
  }
  public void moveChinMotor(double speed){
    chinMotor.set(speed);
  }

  public double getSpinEncoder(){
    return spinEncoder.getPosition();
  }
  public double getChinEncoder(){
    return chinEncoder.getPosition();
  }
  
  public static Intake getInstance() {
    if(instance == null){
      return new Intake();
    }
    return instance;
  }
}
