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
  static TelescopicArm m_Arm;
  
  static Intake m_Intake;
  private static Elevator instance;
  public Elevator(TelescopicArm m_Arm, Intake m_Intake) {
    this.m_Arm = m_Arm;
    this.m_Intake = m_Intake;
    addRequirements(m_Arm, m_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public void CloseIntake(){
    if(m_Intake.getState().toString().equals("OPEN")){
      m_Intake.closeIntake();
    }
    else{
      m_Intake.stopIntakeMotor();
    }
  }

  public void OpenIntake(){
    if(m_Intake.getState().toString().equals("CLOSED")){
      m_Intake.openIntake();
    }
    else{
      m_Intake.stopIntakeMotor();
    }
  }

  public void alignToIntake(){
  }
  // public void topScore(){
  //   if(m_Arm.getWinchState().toString().equals())
  //   m_Arm.(Constants.Elevator.TICKS_TO_TOP);

  public void midScore(){
    // m_Arm.extendRetractArm(Constants.Elevator.TICKS_TO_MID);
  }

  
  public void lowScore(){
    // m_Arm.extendRetractArm(Constants.Elevator.TICKS_TO_BOTTOM);
  }

  public void turn180Degrees(){
    // m_Arm.extendRetractArm(0);
    // m_Arm.turn180Degrees();
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
  public static Elevator getInstance(){
    if(instance == null){

      return new Elevator(m_Arm, m_Intake);
    }
    return null;
    // return null;
  }
}
