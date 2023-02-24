// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopicArm;
import frc.robot.subsystems.Intake;

public class intakeCommand extends CommandBase {
  /** Creates a new intakeCommand. */
  int ticks;
  Intake m_Intake;
  boolean isAtPosition = false;
  PIDController pidController = new PIDController(Constants.Intake.PID[0], Constants.Intake.PID[1], Constants.Intake.PID[2]);

  public intakeCommand(int ticks, Intake m_Intake) {
    this.ticks = ticks;
    this.m_Intake = m_Intake;
    addRequirements(m_Intake);
    //WITH PID
    // pidController.setSetpoint(ticks);
    //pidController.setTolerance(Constants.Intake.INTAKE_TOLERANCE);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_Intake.getIntakeEncoder() - Constants.Intake.TICKS_TO_OPEN) < 5){
      m_Intake.moveIntake(0);
      isAtPosition = true;
    }
    else{
      m_Intake.intakeToBam(Constants.Intake.TICKS_TO_OPEN);
    }

    //WITH PID:

    // m_Intake.moveIntake(pidController.calculate(m_Intake.getIntakeEncoder()));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.stopIntakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAtPosition;
    //WITH PID
    //return pidController.atSetpoint();
  }
}
