// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Auto.AutoPaths.autoChooser;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;

public class RobotContainer {
  private final XboxController driver = new XboxController(1);
  
  private final Intake m_intake = new Intake();

  private final LEDs leds = new LEDs(m_intake);

  private final Swerve s_Swerve = new Swerve(leds);
  
  private final TelescopicArm m_arm = new TelescopicArm();
  private final autoChooser chooser = new autoChooser(m_arm, m_intake, s_Swerve);

  public RobotContainer() {

    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, () -> driver.getLeftBumper()));
    
    configureBindings();
  }

  private void configureBindings() {
    new Trigger(driver::getAButton).onTrue(runOnce(s_Swerve::resetEveything));
  }


  public Command getAutonomousCommand() {
    return chooser.getManualPath();
  }
}
