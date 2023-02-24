// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopicArm;

public class winchCommand extends CommandBase {
  /** Creates a new winchCommand. */
  double ticks;
  TelescopicArm m_Arm;
  boolean isAtPosition;

  public winchCommand(double ticks, TelescopicArm m_Arm) {
    this.ticks = ticks;
    this.m_Arm = m_Arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_Arm.getWinchEncoder() - ticks) < 1){
      isAtPosition = true;
    }
    else{
      m_Arm.winchToBam(ticks);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
