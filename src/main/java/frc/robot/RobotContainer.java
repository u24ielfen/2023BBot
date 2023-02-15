// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AlignToRamp;
import frc.robot.commands.AlignWithNode;
import frc.robot.commands.MoveToTag;
import frc.robot.commands.TeleopSwerve;
// import frc.robot.commands.Elevator.intakeCommand;
import frc.robot.commands.Elevator.winchCommand;
// import frc.robot.commands.autos.exampleAuto;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;
import frc.robot.subsystems.Vision;
// import frc.robot.subsystems.poseEstimator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController driver = new XboxController(1);
  private final XboxController copilot = new XboxController(3);
  
  private final PhotonCamera camera = new PhotonCamera("cam");
  private final Swerve s_Swerve = new Swerve();
  // private final poseEstimator estimator = new poseEstimator(camera, s_Swerve);

  // private final MoveToTag moveTo = new MoveToTag(camera, s_Swerve, estimator::getCurrentPose);

  // private final exampleAuto auto = new exampleAuto(s_Swerve);
  private final AlignToRamp alignToRamp = new AlignToRamp(s_Swerve);
  private final Limelight limelight = new Limelight();
  private final AlignWithNode alignNode = new AlignWithNode(limelight, s_Swerve);
  private final Vision m_Vision = new Vision(camera);
  // private final Intake m_intake = new Intake();
  // private final intakeCommand c_MoveIntake = new intakeCommand(null);
  private final TelescopicArm m_arm = new TelescopicArm();
  private final winchCommand c_WinchCommand = new winchCommand(m_arm);
  
  public RobotContainer() {

    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, () -> driver.getLeftBumper()));
    
    configureBindings();
  }

  private void configureBindings() {
    
    //pilot controls
    // new Trigger(driver::getAButton).onTrue(runOnce(estimator::resetFieldPosition));
    new Trigger(driver::getRightBumper).whileTrue(alignToRamp);
    // new Trigger(driver::getRightBumper).whileTrue(alignNode);

    //copilot controls
    // new Trigger(copilot::getLeftBumper).onTrue(runOnce(c_MoveIntake::openIntake));
    // new Trigger(copilot::getRightBumper).onTrue(runOnce(c_MoveIntake::closeIntake));
    new Trigger(copilot::getAButton).onTrue(runOnce(m_arm::zeroWinch));
    //test controls
  }

  public Command getAutonomousCommand() {
    
return null;
    // return autoBuilder.fullAuto(testPath);
  }
}
