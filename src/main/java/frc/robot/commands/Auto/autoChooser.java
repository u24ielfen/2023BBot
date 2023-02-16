// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class autoChooser extends CommandBase {
  /** Creates a new autoChooser. */
  static autoChooser instance;
  SendableChooser<String> chooser = new SendableChooser<>();
  
  public autoChooser() {
    chooser.setDefaultOption("Nothing", "Nothing");
    chooser.addOption("TopCone_5__Balance", "TopCone_5__Balance");
    SmartDashboard.putData(chooser);
  }

  public String getAutonomous(){
    return chooser.getSelected();
  }

  public static autoChooser getInstance(){
    if(instance == null){
      return new autoChooser();
    }
    return instance;
  }
}
