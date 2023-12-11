// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.AutoCommands.Cone;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class TopCone extends CommandBase {
  /** Creates a new ConeToTop. */
  static TopCone instance;
  static TelescopicArm m_arm;
  boolean isFinished = false;;
  static Intake m_intake;
  static Swerve s_Swerve;
  double time;
  double distanceToGo;
  public TopCone(TelescopicArm m_arm, Intake m_intake, Swerve s_Swerve, double time) {
    this.m_arm = m_arm;
    this.s_Swerve = s_Swerve;
    this.m_intake = m_intake;
    this.time = time;
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
    return new SequentialCommandGroup(new pivotCommand(0.574, 0.6, m_arm).andThen
    (new winchCommand(146, 0.7, m_arm).
    andThen(new intakeCommand(m_intake, "Out").
    andThen(new winchCommand(-4, 0.7, m_arm)).andThen(new driveXDistance(s_Swerve, time, "Forward")))));
    // return null;
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
