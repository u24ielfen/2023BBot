// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TelescopicArm;
import frc.robot.subsystems.Vision;

public class AlignToGamePiece extends CommandBase {
  /** Creates a new AlignToGamePiece. */
  Intake m_intake;
  TelescopicArm m_TelescopicArm;
  Vision m_Vision;
  Boolean LookAtCone;

  public AlignToGamePiece(Intake m_intake, TelescopicArm m_TelescopicArm, Vision m_Vision, Boolean LookAtCone) {
    this.LookAtCone = LookAtCone;
    this.m_intake = m_intake;
    this.m_Vision = m_Vision;
    this.m_TelescopicArm = m_TelescopicArm;
    addRequirements(m_intake, m_TelescopicArm, m_Vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.LookAtCone){
      m_Vision.photonSetPipeline(m_Vision.getCamera(), Constants.VisionConstants.conePipeline);

    }
    else if(!this.LookAtCone){
      m_Vision.photonSetPipeline(m_Vision.getCamera(), Constants.VisionConstants.conePipeline);
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
