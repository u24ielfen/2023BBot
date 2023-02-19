// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.Winch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.TelescopicArm;

public class winchToBottom extends CommandBase {
  /** Creates a new winchToTop. */
  static TelescopicArm m_Arm;
  static winchToBottom instance;

  boolean isAtPosition = false;
  public winchToBottom(TelescopicArm m_Arm) {
    this.m_Arm = m_Arm;
    addRequirements(m_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void execute() {
    if(Math.abs(m_Arm.getWinchEncoder() - Constants.Elevator.WINCH_TICKS_TO_BOTTOM) < 5){
      isAtPosition = true;
    }
    else{
      m_Arm.winchToBam(Constants.Elevator.WINCH_TICKS_TO_BOTTOM);
    }
  }
  public static winchToBottom getInstance(){
    if(instance == null){
      return new winchToBottom(m_Arm);
    }
    return instance;
  }

  @Override
  public boolean isFinished() {
    return isAtPosition;
  }
}
