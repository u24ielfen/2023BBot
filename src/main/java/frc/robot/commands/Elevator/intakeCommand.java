// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class intakeCommand extends CommandBase {
  /** Creates a new intakeCommand. */
  Intake m_Intake;
  boolean isAtPosition = false;
  String inOut;
  double startingTime;
  PIDController pidController = new PIDController(Constants.Intake.PID[0], Constants.Intake.PID[1], Constants.Intake.PID[2]);

  public intakeCommand(Intake m_Intake, String inOut) {
    this.inOut = inOut;
    this.m_Intake = m_Intake;
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(inOut.equals("In")){
      m_Intake.moveSpinMotor(0.8);
    }
    else if(inOut.equals("Out")){
      m_Intake.moveSpinMotor(-0.8);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.stopSpinMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startingTime > .8;
  }
}
