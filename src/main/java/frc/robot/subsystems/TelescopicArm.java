// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopicArm extends SubsystemBase {
  /** Creates a new TelescopicArm. */
  CANSparkMax pivotMotor = new CANSparkMax(Constants.Elevator.PIVOT_MOTORID, MotorType.kBrushless);
  RelativeEncoder pivotEncoder;

  CANSparkMax winchMotor = new CANSparkMax(Constants.Elevator.EXTENDING_MOTORID, MotorType.kBrushless);
  RelativeEncoder winchEncoder;

  PIDController winchPidController;

  public enum states{}

  public TelescopicArm() {
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

    winchPidController.setP(Constants.Elevator.ELEVATOR_PID[0]);
    winchPidController.setI(Constants.Elevator.ELEVATOR_PID[1]);
    winchPidController.setD(Constants.Elevator.ELEVATOR_PID[2]);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveMotor(double setPoint){
    winchPidController.setSetpoint(setPoint);
    double speed = winchPidController.calculate(winchEncoder.getPosition());
    winchMotor.set(speed);
  }
  
}
