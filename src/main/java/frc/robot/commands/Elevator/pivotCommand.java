// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopicArm;

public class pivotCommand extends CommandBase {
  /** Creates a new pivotCommand. */
  TelescopicArm m_Arm;
  double ticks;
  boolean isAtPosition;
  PIDController pidController = new PIDController(Constants.Elevator.PIVOT_PID[0], Constants.Elevator.PIVOT_PID[1], Constants.Elevator.PIVOT_PID[2]);
  double maxSpeed;

  public pivotCommand(double ticks, double maxSpeed, TelescopicArm m_Arm) {
    this.ticks = ticks;
    this.maxSpeed = maxSpeed;
    this.m_Arm = m_Arm;
    addRequirements(m_Arm);

    //WITH PID:
    // pidController.setSetpoint(ticks);
    // pidController.setTolerance(Constants.Elevator.PIVOT_TOLERANCE);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("PIVOT RUNNING", true);
      m_Arm.pivotToBam(ticks, maxSpeed);

    //WITH PID:
    // m_Arm.movePivot(pidController.calculate(m_Arm.getPivotEncoder()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("PIVOT RUNNING", false);
    return Math.abs(m_Arm.getPivotEncoder() - ticks) < 0.001;
    //WITH PID
    //return pidController.atSetpoint();
  }
}
