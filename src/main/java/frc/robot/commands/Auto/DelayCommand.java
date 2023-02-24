// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DelayCommand extends CommandBase {
  /** Creates a new DelayCommand. */
  double time;
  String action;
  public DelayCommand(double time, String action) {
    this.time = time;
    this.action = action;
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean(action, true);
    new WaitCommand(time).schedule();
    SmartDashboard.putString("Wait Command", action);
    SmartDashboard.putBoolean(action, false);

  }
}
