// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.Pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.TelescopicArm;

public class pivotToBottom extends CommandBase {
  /** Creates a new winchToTop. */
  static TelescopicArm m_Arm;
  boolean isAtPosition = false;

  static pivotToBottom instance;

  public pivotToBottom(TelescopicArm m_Arm) {
    this.m_Arm = m_Arm;

    addRequirements(m_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void execute() {
    
    if(Math.abs(m_Arm.getPivotEncoder() - Constants.Elevator.PIVOT_TICKS_TO_BOTTOM) < 5){
      isAtPosition = true;
    }
    else{
      m_Arm.pivotToBam(Constants.Elevator.PIVOT_TICKS_TO_BOTTOM);
    }
  }

  public static pivotToBottom getInstance(){
    if(instance == null){
      return new pivotToBottom(m_Arm);
    }
    return instance;
  }

  @Override
  public boolean isFinished() {
    return isAtPosition;
  }
}
