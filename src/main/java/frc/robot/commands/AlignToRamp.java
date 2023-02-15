// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AlignToRamp extends CommandBase {
  /** Creates a new AlignToRamp. */
  Swerve s_Swerve;
  PIDController pidController;
  public AlignToRamp(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    pidController = new PIDController(0.1, 0, 0);
    pidController.setTolerance(1);
    addRequirements(s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationVal = pidController.calculate(s_Swerve.getPitch().getDegrees(), 0);
    s_Swerve.drive(new Translation2d(-translationVal,0).times(2), 0.0, false);
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
