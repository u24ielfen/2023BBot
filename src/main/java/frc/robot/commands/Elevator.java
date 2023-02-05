// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.invoke.ConstantBootstraps;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TelescopicArm;

public class Elevator extends CommandBase {
  /** Creates a new Elevator. */
  TelescopicArm m_Arm;
  
  Intake m_Intake;
  public Elevator(TelescopicArm m_Arm, Intake m_Intake) {
    this.m_Arm = m_Arm;
    this.m_Intake = m_Intake;
    addRequirements(m_Arm, m_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public Command CloseIntake(){
    return new InstantCommand(() -> m_Intake.closeIntake());
  }

  public Command OpenIntake(){
    return new InstantCommand(() -> m_Intake.openIntake());

  }

  public Command alignToIntake(){
    return new SequentialCommandGroup(OpenIntake(), /*thing for aligning */ CloseIntake());
  }
  public Command topScore(){
    
    m_Arm.moveMotor(Constants.Elevator.TICKS_TO_TOP);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
