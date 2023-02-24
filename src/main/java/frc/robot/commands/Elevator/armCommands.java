// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.MoveToPose;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;
import frc.robot.subsystems.poseEstimator;
import frc.robot.subsystems.poseToTag;


public class armCommands extends CommandBase {
  /** Creates a new armCommands. */

  TelescopicArm m_Arm = TelescopicArm.getInstance();
  //INTAKE:
  Intake m_Intake = new Intake();
  intakeCommand c_openIntake = new intakeCommand(Constants.Intake.OPEN_POSITION, m_Intake);
  intakeCommand c_closeIntake = new intakeCommand(Constants.Intake.CLOSED_POSITION, m_Intake);
  //PIVOT:
  pivotCommand c_pivotToBottom = new pivotCommand(Constants.Elevator.PIVOT_TICKS_TO_BOTTOM, m_Arm);
  pivotCommand c_pivotToMid = new pivotCommand(Constants.Elevator.PIVOT_TICKS_TO_MID, m_Arm);
  pivotCommand c_pivotToTop = new pivotCommand(Constants.Elevator.PIVOT_TICKS_TO_TOP, m_Arm);
    //inveted:
  pivotCommand c_I_pivotToBottom = new pivotCommand(Constants.Elevator.PIVOT_TICKS_TO_BOTTOM_INVERTED, m_Arm);
  pivotCommand c_I_pivotToMid = new pivotCommand(Constants.Elevator.PIVOT_TICKS_TO_MID_INVERTED, m_Arm);
  pivotCommand c_I_pivotToTop = new pivotCommand(Constants.Elevator.PIVOT_TICKS_TO_TOP_INVERTED, m_Arm);

  //WINCH:
  winchCommand c_winchToBottom = new winchCommand(Constants.Elevator.WINCH_TICKS_TO_BOTTOM , m_Arm);
  winchCommand c_winchToMid = new winchCommand(Constants.Elevator.WINCH_TICKS_TO_MID ,m_Arm);
  winchCommand c_winchToTop = new winchCommand(Constants.Elevator.WINCH_TICKS_TO_TOP ,m_Arm);
  poseToTag s_poseToTag = poseToTag.getInstance();
  Swerve s_Swerve = Swerve.getInstance();
  poseEstimator estimator = poseEstimator.getInstance();
  //Work?
  MoveToPose c_moveTo = new MoveToPose(s_poseToTag.getPhotonCam(), s_Swerve, poseEstimator.getCurrentPose(), s_poseToTag.getPose());

  boolean finished = false;

  String action;
  public armCommands(String action) {
    this.action = action;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (action) {
      case "Arm To Top":
        armToTop();
        break;
      case "Arm To Mid":
        armToMid();
        break;
      case "Arm To Bottom":
        armToBottom();
      case "Inverted Arm To Top":
        I_armToTop();
        break;
      case "Inverted Arm To Mid":
        I_armToMid();
        break;
      case "Inverted Arm To Bottom":
        I_armToBottom();
        break;
      case "Inverted Pick Up":
        I_pickUpObject();
        break;
      case "Pick Up":
        pickUpObject();
        break;
      }
  }

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

  
  public Command I_pickUpObject(){
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
