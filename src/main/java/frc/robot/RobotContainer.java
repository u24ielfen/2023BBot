// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToRamp;
import frc.robot.commands.MoveToTag;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autos.exampleAuto;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...
  private final XboxController driver = new XboxController(1);
  /* Driver Buttons */
  PhotonCamera camera = new PhotonCamera("cam");
  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  exampleAuto auto = new exampleAuto(s_Swerve);
  AlignToRamp alignToRamp = new AlignToRamp(s_Swerve);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  private final MoveToTag moveTo = new MoveToTag(camera, s_Swerve, s_Swerve.getPose()); 
  // Replace with CommandPS4Controller or CommandJoystick if needed
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, robotCentric.getAsBoolean()));
    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
    new Trigger(driver::getAButton).onTrue(s_Swerve.resetFieldPos());
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    new Trigger(driver::getBButton).whileTrue(moveTo);
    new Trigger(driver::getRightBumper).whileTrue(alignToRamp);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return new exampleAuto(s_Swerve);
  }
}
