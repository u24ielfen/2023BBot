// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopicArm;

public class pivotCommand extends CommandBase {
  /** Creates a new pivotCommand. */
  static TelescopicArm m_Arm;
  PIDController controller = new PIDController(Constants.Elevator.PIVOT_PID[0], Constants.Elevator.PIVOT_PID[1], Constants.Elevator.PIVOT_PID[2]);
  boolean isAtPosition = false;
  static pivotCommand instance;

  
  double pivotTicksToTop = Constants.Elevator.PIVOT_TICKS_TO_TOP;
  double pivotTicksToMid = Constants.Elevator.PIVOT_TICKS_TO_MID;
  double pivotTicksToBottom = Constants.Elevator.PIVOT_TICKS_TO_BOTTOM;

  public pivotCommand(TelescopicArm m_Arm) {
    controller.setTolerance(3);
    pivotCommand.m_Arm = m_Arm;
    addRequirements(m_Arm);
    instance = new pivotCommand(m_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isAtPosition = false;
  }

  public void pivotToBottom(){
    controller.setSetpoint(Constants.Elevator.PIVOT_TICKS_TO_BOTTOM);
    m_Arm.movePivot(controller.calculate(m_Arm.getPivotEncoder()));
    if(controller.atSetpoint()){
      isAtPosition = true;
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  public void pivotToMiddle(){
    controller.setSetpoint(Constants.Elevator.PIVOT_TICKS_TO_MID);
    m_Arm.movePivot(controller.calculate(m_Arm.getPivotEncoder()));
    if(controller.atSetpoint()){
      isAtPosition = true;
    }
  }

  public void pivotToTop(){
    controller.setSetpoint(Constants.Elevator.PIVOT_TICKS_TO_TOP);
    m_Arm.movePivot(controller.calculate(m_Arm.getPivotEncoder()));
    if(controller.atSetpoint()){
      isAtPosition = true;
    }
  }
  
  public void pivotToTopBam(){
    pivotToBam(pivotTicksToTop);
  }

  public void pivotToMidBam(){
    pivotToBam(pivotTicksToMid);
  }

  public void pivotToBottomBam(){
    pivotToBam(pivotTicksToBottom);
  }

  public void pivotToBam(double ticks){
    if(m_Arm.getPivotEncoder() < ticks - 20){
      m_Arm.movePivot(0.8);
    }
    else if(m_Arm.getPivotEncoder() > ticks - 20 && m_Arm.getPivotEncoder() < ticks){
      m_Arm.movePivot(0.3);
    }
    else if(m_Arm.getPivotEncoder() > ticks + 20){
      m_Arm.movePivot(-0.8);
    }
    else if(m_Arm.getPivotEncoder() < ticks + 20 && m_Arm.getPivotEncoder() > ticks){
      m_Arm.movePivot(-0.3);
    }
    else{
      m_Arm.movePivot(0);
      isAtPosition = true;
    }
  }

  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAtPosition;
  }

  public static pivotCommand getInstance(){
    if(instance == null){
      return new pivotCommand(m_Arm);
    }
    return instance;
  }
}
