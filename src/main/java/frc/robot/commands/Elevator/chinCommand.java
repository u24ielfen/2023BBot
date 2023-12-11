// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class chinCommand extends CommandBase {
  /** Creates a new chinCommand. */
  Intake m_Intake;
  double startingPos = 0;
  double ticks;
  public chinCommand(double ticks, Intake m_Intake) {
    this.ticks = ticks;
    this.m_Intake = m_Intake;
    addRequirements(m_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPos = m_Intake.getChinEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.chinTicksToBam(ticks);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_Intake.getChinEncoder() - ticks) < 0.2 ;
  }
}
