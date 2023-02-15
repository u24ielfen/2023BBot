// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopicArm;
public class winchCommand extends CommandBase {
  /** Creates a new winchCommand. */
  static TelescopicArm m_Arm;
  double ticksToTop = Constants.Elevator.PIVOT_TICKS_TO_TOP;
  boolean isAtPosition = false;
  PIDController controller = new PIDController(Constants.Elevator.WINCH_PID[0], Constants.Elevator.WINCH_PID[1], Constants.Elevator.WINCH_PID[2]);

  double winchTicksToTop = Constants.Elevator.WINCH_TICKS_TO_TOP;
  double winchTicksToMid = Constants.Elevator.WINCH_TICKS_TO_MID;
  double winchTicksToBottom = Constants.Elevator.WINCH_TICKS_TO_BOTTOM;

  // static winchCommand instance;

  public winchCommand(TelescopicArm m_Arm) {
    winchCommand.m_Arm = m_Arm;
    controller.setTolerance(3);
    addRequirements(m_Arm);
    // instance = new winchCommand(m_Arm);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isAtPosition = false;
  }

  public void winchToBottom(){
    controller.setSetpoint(Constants.Elevator.WINCH_TICKS_TO_BOTTOM);
    m_Arm.moveWinch(controller.calculate(m_Arm.getWinchEncoder()));
    if(controller.atSetpoint()){
      m_Arm.moveWinch(0);
      isAtPosition = true;
    }
  }

  public void winchToMiddle(){
    controller.setSetpoint(Constants.Elevator.WINCH_TICKS_TO_MID);
    m_Arm.moveWinch(controller.calculate(m_Arm.getWinchEncoder()));
    if(controller.atSetpoint()){
      m_Arm.moveWinch(0);
      isAtPosition = true;
    }
  }

  public void winchToTop(){
    controller.setSetpoint(Constants.Elevator.WINCH_TICKS_TO_TOP);
    m_Arm.moveWinch(controller.calculate(m_Arm.getWinchEncoder()));
    if(controller.atSetpoint()){
      m_Arm.moveWinch(0);
      isAtPosition = true;
    }
  }
  
  public void winchToTopBam(){
    winchToBam(winchTicksToTop);
  }

  public void winchToMidBam(){
    winchToBam(winchTicksToMid);
  }

  public void winchToBottomBam(){
    winchToBam(winchTicksToBottom);
  }

  public void winchToBam(double ticks){
    if(m_Arm.getWinchEncoder() < ticks - 20){
      m_Arm.moveWinch(0.8);
    }
    else if(m_Arm.getWinchEncoder() > ticks - 20 && m_Arm.getWinchEncoder() < ticks){
      m_Arm.moveWinch(0.4);
    }
    else if(m_Arm.getWinchEncoder() > ticks){
      m_Arm.moveWinch(-0.3);
    }
    else{
      m_Arm.moveWinch(0);
      isAtPosition = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAtPosition;
  }

  public static winchCommand getInstance(){
    // if(instance == null){
    //   return new winchCommand(m_Arm);
    // }
    return new winchCommand(m_Arm);
  }
}
