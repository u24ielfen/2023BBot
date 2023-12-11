// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.AutoPaths;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.commands.Auto.AutoCommands.driveXDistance;
import frc.robot.commands.Auto.AutoCommands.Cone.LowCone;
import frc.robot.commands.Auto.AutoCommands.Cone.MidCone;
import frc.robot.commands.Auto.AutoCommands.Cone.TopCone;
import frc.robot.commands.Auto.AutoCommands.Cube.LowCube;
import frc.robot.commands.Auto.AutoCommands.Cube.MidCube;
import frc.robot.commands.Auto.AutoCommands.Cube.TopCube;
import frc.robot.commands.Elevator.intakeCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;

public class autoChooser extends CommandBase {
  /** Creates a new autoChooser. */
  static autoChooser instance;
  SendableChooser<Command> manualChooser = new SendableChooser<>();
  SendableChooser<Command> manualBackup = new SendableChooser<>();

  SendableChooser<Command> pathChooser = new SendableChooser<>();
  
  static TelescopicArm m_arm;
  static Intake m_intake;
  static Swerve s_Swerve;
  double distanceToGo;
  @Override
  
  public void initialize() {
    distanceToGo = SmartDashboard.getNumber("Time To Go", 2.7);

  }

  public autoChooser(TelescopicArm m_arm, Intake m_intake, Swerve s_Swerve) {
    addRequirements(m_arm, m_intake, s_Swerve);

    manualChooser.setDefaultOption("Nothing Manual", new WaitCommand(15));
    manualChooser.addOption("NO AUTO", new WaitCommand(15));
    manualChooser.addOption("Top Cone", new TopCone(m_arm, m_intake, s_Swerve, 0));
    manualChooser.addOption("Mid Cone", new MidCone(m_arm, m_intake, s_Swerve, 0));
    manualChooser.addOption("Low Cone", new LowCone(m_arm, m_intake, s_Swerve, 0));
    manualChooser.addOption("Top Cube", new TopCube(m_arm, m_intake, s_Swerve, 0));
    manualChooser.addOption("Mid Cube", new MidCube(m_arm, m_intake, s_Swerve, 0));
    manualChooser.addOption("Low Cube", new LowCube(m_arm, m_intake, s_Swerve, 0));
    manualChooser.addOption("Top Cone CHARGE", new TopCone(m_arm, m_intake, s_Swerve, 2.7));
    manualChooser.addOption("Mid Cone CHARGE", new MidCone(m_arm, m_intake, s_Swerve, 2.7));
    manualChooser.addOption("Low Cone CHARGE", new LowCone(m_arm, m_intake, s_Swerve, 2.7));
    manualChooser.addOption("Top Cube CHARGE", new TopCube(m_arm, m_intake, s_Swerve, 2.7));
    manualChooser.addOption("Mid Cube CHARGE", new MidCube(m_arm, m_intake, s_Swerve, 2.7));
    manualChooser.addOption("Low Cube CHARGE", new LowCube(m_arm, m_intake, s_Swerve, 2.7));
    manualChooser.addOption("Top Cone FAR", new TopCone(m_arm, m_intake, s_Swerve, 5));
    manualChooser.addOption("Mid Cone FAR", new MidCone(m_arm, m_intake, s_Swerve, 5));
    manualChooser.addOption("Low Cone FAR", new LowCone(m_arm, m_intake, s_Swerve, 5));
    manualChooser.addOption("Top Cube FAR", new TopCube(m_arm, m_intake, s_Swerve, 5));
    manualChooser.addOption("Mid Cube FAR", new MidCube(m_arm, m_intake, s_Swerve, 5));
    manualChooser.addOption("Low Cube FAR", new LowCube(m_arm, m_intake, s_Swerve, 5));

    manualChooser.addOption("FAR BACKUP", new driveXDistance(s_Swerve, 5,  "Forward"));
    manualChooser.addOption("CHARGE BACKUP", new driveXDistance(s_Swerve, 2.8,  "Forward"));
    manualBackup.setDefaultOption("No Backup", new WaitCommand(15)); // TOP CONE ^
    manualBackup.addOption("Far Backup", new driveXDistance(s_Swerve, 4, "Forward"));
    manualBackup.addOption("Charge Station (Bang)", new SequentialCommandGroup(new intakeCommand(m_intake, "Out").andThen(new driveXDistance(s_Swerve, 5, "Forward"))));
    
    //PathPlanner paths
    pathChooser.setDefaultOption("Nothing Generated", new WaitCommand(15));
    pathChooser.addOption("Something", new WaitCommand(15));

    SmartDashboard.putData(manualChooser);
    SmartDashboard.putData(manualBackup);
      SmartDashboard.putData(pathChooser);
  }

  public Command getManualPath(){
    return manualChooser.getSelected();
  }
  public Command getManualBackup(){
    return manualBackup.getSelected();
  }
  public Command getGeneratedPath(){
    return pathChooser.getSelected();
  }

  public String getGeneratedPathName(){
    return pathChooser.getSelected().getName();
  }

  public String getManualPathName(){
    return manualChooser.getSelected().getName();
  }

  public String getManualBackupName(){
    return manualBackup.getSelected().getName();
  }

  public static autoChooser getInstance(){
    if(instance == null){
      return new autoChooser(m_arm, m_intake, s_Swerve);
    }
    return instance;
  }
}
