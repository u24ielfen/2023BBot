// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.Elevator.Intake.closeIntake;
import frc.robot.commands.Elevator.Intake.openIntake;
import frc.robot.commands.Elevator.Pivot.pivotToMid;
import frc.robot.commands.Elevator.Pivot.pivotToTop;
import frc.robot.commands.Elevator.Winch.winchToBottom;
import frc.robot.commands.Elevator.Winch.winchToMid;
import frc.robot.commands.Elevator.Winch.winchToTop;
import frc.robot.commands.Elevator.Pivot.pivotToBottom;
import frc.robot.commands.Elevator.Pivot.I_pivotToMid;
import frc.robot.commands.Elevator.Pivot.I_pivotToTop;
import frc.robot.commands.Elevator.Pivot.I_pivotToBottom;

public class armCommands extends CommandBase {
  /** Creates a new armCommands. */

  //INTAKE:
  closeIntake c_closeIntake = closeIntake.getInstance();
  openIntake c_openIntake = openIntake.getInstance();
  
  //PIVOT:
  pivotToBottom c_pivotToBottom = pivotToBottom.getInstance();
  pivotToMid c_pivotToMid = pivotToMid.getInstance();
  pivotToTop c_pivotToTop = pivotToTop.getInstance();
    //inveted:
  I_pivotToBottom c_I_pivotToBottom = I_pivotToBottom.getInstance();
  I_pivotToMid c_I_pivotToMid = I_pivotToMid.getInstance();
  I_pivotToTop c_I_pivotToTop = I_pivotToTop.getInstance();

  //WINCH:
  winchToBottom c_winchToBottom = winchToBottom.getInstance();
  winchToMid c_winchToMid = winchToMid.getInstance();
  winchToTop c_winchToTop = winchToTop.getInstance();

  MoveToPose c_moveTo = MoveToPose.getInstance();

  boolean finished = false;

  public armCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
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
  
  public Command armToTop(){
    return new SequentialCommandGroup(c_pivotToTop.andThen(c_winchToTop).andThen(c_openIntake).andThen(() -> finishedTrue()));
  }
  
  public Command I_armToTop(){
    return new SequentialCommandGroup(c_I_pivotToTop.andThen(c_winchToTop).andThen(c_openIntake).andThen(() -> finishedTrue()));
  }
  
  public Command armToMid(){
    return new SequentialCommandGroup(c_pivotToMid.andThen(c_winchToMid).andThen(c_openIntake).andThen(() -> finishedTrue()));
  }
  
  public Command I_armToMid(){
    return new SequentialCommandGroup(c_I_pivotToMid.andThen(c_winchToMid).andThen(c_openIntake).andThen(() -> finishedTrue()));
  }
  
  public Command armToBottom(){
    return new SequentialCommandGroup(c_pivotToBottom.andThen(c_winchToBottom).andThen(c_openIntake).andThen(() -> finishedTrue()));
  }
  
  public Command I_armToBottom(){
    return new SequentialCommandGroup(c_I_pivotToBottom.andThen(c_winchToBottom).andThen(c_openIntake).andThen(() -> finishedTrue()));
  }

  public Command pickUpObject(){
    return new SequentialCommandGroup(c_moveTo.andThen());
  }

  public void finishedTrue(){
    finished = true;
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
