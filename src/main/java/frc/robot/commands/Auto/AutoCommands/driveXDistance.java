// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.AutoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class driveXDistance extends CommandBase {
  /** Creates a new driveXDistance. */
  Swerve s_Swerve;
  double distance;
  double initialPose;
  double initialTime;
  double timeNow;
  String direction;
  public driveXDistance(Swerve s_Swerve, double distance, String direction) {
    this.s_Swerve = s_Swerve;
    this.direction = direction;
    this.distance = distance;
    addRequirements(s_Swerve);
    SmartDashboard.putBoolean("Swerve Drive X Running", false);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPose = s_Swerve.getPose().getY();
    initialTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putBoolean("Swerve Drive X Running", true);
    timeNow = Timer.getFPGATimestamp();
    if(direction.equals("Forward")){

      s_Swerve.drive(new Translation2d(0, 0.3), 0, false);
    }
    else if(direction.equals("Backwards")){
      s_Swerve.drive(new Translation2d(0, -0.3), 0, true);

    }
    // SmartDashboard.putNumber("Distance from goal X?", distance - Math.abs(s_Swerve.getPose().getX() - initialPose));
    // SmartDashboard.putNumber("Distance from goal Y?", distance - Math.abs(s_Swerve.getPose().getY() - initialPose));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // SmartDashboard.putBoolean("Swerve Drive X Running", false);
    return (timeNow - initialTime) > distance;
    // return Math.abs(distance - Math.abs(s_Swerve.getPose().getX() - initialPose)) < 0.03;
    // return (Math.abs(Math.abs(s_Swerve.getYGyro()) - distance) < 0.02);
  }
}
