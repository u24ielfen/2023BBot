// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.AutoCommands.Cone;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.Elevator.intakeCommand;
import frc.robot.commands.Elevator.pivotCommand;
import frc.robot.commands.Elevator.winchCommand;
import frc.robot.commands.Auto.AutoCommands.driveXDistance;
import frc.robot.subsystems.TelescopicArm;

public class LowCone extends CommandBase {
  /** Creates a new ConeToTop. */
  static TelescopicArm m_arm;
  boolean isFinished = false;;
  double time;
  static Intake m_intake;
  static Swerve s_Swerve;
  static LowCone instance;
  public LowCone(TelescopicArm m_arm, Intake m_intake, Swerve s_Swerve, double time) {
    this.m_arm = m_arm;
    this.s_Swerve = s_Swerve;
    this.time = time;
    this.m_intake = m_intake;

    addRequirements(m_arm, m_intake, s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getCommand().schedule();
  }

  public Command getCommand(){
    //TODO: FIND VALUES
    // return new SequentialCommandGroup(new pivotCommand(0.36, m_arm).alongWith
    // (new winchCommand(180, m_arm).andThen(new intakeCommand(-10, m_intake).
    // andThen(new winchCommand(15, m_arm)).andThen(new InstantCommand(this::trueFinished)))));
return null;
  }

  public void trueFinished(){
    isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
