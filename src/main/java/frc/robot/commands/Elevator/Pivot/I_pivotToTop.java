// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.Pivot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.TelescopicArm;

public class I_pivotToTop extends CommandBase {
  /** Creates a new winchToTop. */
  static TelescopicArm m_Arm;
  boolean isAtPosition = false;
  static boolean isInverted = false;

  static I_pivotToTop instance;

  public I_pivotToTop(TelescopicArm m_Arm) {
    this.m_Arm = m_Arm;

    addRequirements(m_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void execute() {
    if(Math.abs(m_Arm.getPivotEncoder() - Constants.Elevator.PIVOT_TICKS_TO_TOP_INVERTED) < 5){
      isAtPosition = true;
    }
    else{
      m_Arm.pivotToBam(Constants.Elevator.PIVOT_TICKS_TO_TOP_INVERTED);
    }
  }

  public static I_pivotToTop getInstance(){
    if(instance == null){
      return new I_pivotToTop(m_Arm);
    }
    return instance;
  }
  @Override
  public boolean isFinished() {
    return isAtPosition;
  }
}
