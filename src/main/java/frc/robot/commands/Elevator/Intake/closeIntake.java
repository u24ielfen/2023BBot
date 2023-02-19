// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.Intake;

public class closeIntake extends CommandBase {
  /** Creates a new openIntake. */
  static Intake m_Intake;
  public static closeIntake instance;
  boolean isAtPosition = false;
  public closeIntake(Intake m_Intake) {
    this.m_Intake = m_Intake;
    addRequirements(m_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_Intake.getIntakeEncoder() - Constants.Intake.TICKS_TO_CLOSE) < 5){
      m_Intake.moveIntake(0);
      isAtPosition = true;
    }
    else{
      m_Intake.intakeToBam(Constants.Intake.TICKS_TO_CLOSE);
    }

  }

  public static closeIntake getInstance(){
    if(instance == null){
      return new closeIntake(m_Intake);
    }
    return instance;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.stopIntakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
