// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.poseEstimator;

public class photon_MoveToPose extends CommandBase {
  /** Creates a new photon_MoveToPose. */
  PathPlannerTrajectory trajectory;
  poseEstimator estimator;
  Swerve s_Swerve;
  
  public photon_MoveToPose(poseEstimator estimator, Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    this.estimator = estimator;
    addRequirements(estimator, s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trajectory = PathPlanner.generatePath(
      new PathConstraints(0, 0), 
      new PathPoint(estimator.getCurrentPose().getTranslation(), s_Swerve.getGyro()), 
      new PathPoint(null, null), new PathPoint(null, null, null));
  
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
