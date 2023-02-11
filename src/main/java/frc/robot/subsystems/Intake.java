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

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.MOTORID, MotorType.kBrushless);

  RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  PIDController intakePID = new PIDController(Constants.Intake.PID[0], Constants.Intake.PID[1], Constants.Intake.PID[2]);
  public enum state{
    OPEN,
    CLOSED
  }
  public state currentState;
  public Intake() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakePID.setTolerance(2);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void zeroIntake(){
      intakeEncoder.setPosition(0);
  }

  public void openIntake(){
    intakePID.setSetpoint(Constants.Intake.OPEN_POSITION);
    intakeMotor.set(intakePID.calculate(intakeEncoder.getPosition()));
    if(intakePID.atSetpoint()){
      intakeMotor.set(0);
      currentState = state.OPEN;
    }
  }

  public void closeIntake(){
    intakePID.setSetpoint(Constants.Intake.CLOSED_POSITION);
    intakeMotor.set(intakePID.calculate(intakeEncoder.getPosition()));
    if(intakePID.atSetpoint()){
      intakeMotor.set(0);
      currentState = state.CLOSED;
    }
  }

  public state getState(){
    return currentState;
  } 

  public void stopIntakeMotor(){
    intakeMotor.set(0);

  }
}
