// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;


public class AlignWithNode extends CommandBase {
  /** Creates a new AlignWithNode. */
  Limelight limelight;
  Swerve s_Swerve;
  PIDController controllerX = new PIDController(0, 0, 0, 1); 
  PIDController controllerY;
  double xVal = 0.0;
  double yVal = 0.0;
  public AlignWithNode(Limelight limelight, Swerve s_Swerve) {
    this.limelight = limelight;
    this.s_Swerve = s_Swerve;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controllerX = new PIDController(2, 0, 0);
    controllerY = new PIDController(2, 0, 0);
    controllerX.setTolerance(0.5);
    controllerY.setTolerance(0.5);
    xVal = controllerX.calculate(limelight.getAngleX());
    makeMinMax(xVal);

    yVal = controllerX.calculate(limelight.getDistance(LimelightConstants.midConeHeight));
    makeMinMax(yVal);
    Translation2d movement = new Translation2d(yVal, xVal);
    //FIX: make sure ^^ works (switch yVal and xVal?)
    s_Swerve.drive(movement, 0.0, true);
    
  }

  public void makeMinMax(double value){
    value = Math.min(value, -1);
    value = Math.max(value, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.hasTarget()){
      controllerY.setSetpoint(LimelightConstants.setPointDisY);
      controllerX.setSetpoint(LimelightConstants.alignedConeX);
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
