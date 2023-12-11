// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  static LEDs instance;

  DigitalOutput blueLEDs = new DigitalOutput(3);
  DigitalOutput greenLEDs = new DigitalOutput(4);
  static Swerve s_Swerve;
  boolean on = false;
  static Intake m_intake;
  public LEDs(Intake m_intake) {
    this.m_intake = m_intake;
  }
    
  public static LEDs getInstance(){
    if(instance == null){
      return new LEDs(m_intake);
    }
    return instance;
  }
  // public void thingON(){

  //     blueLEDs.set(false); //turns leds on
  // }
  // public void thingOFF(){
  //   blueLEDs.set(true); //turns leds off

  // }

  public void toggleBrake(){
    if(on){
      on = false;
    }
    else{ on  = true;}
  }

  @Override
  public void periodic() {
    if(m_intake.getLimitSwitch()){
      greenLEDs.set(false); //turns LEDs on
    }else greenLEDs.set(true); //turns LEDs off
    if(on){
      blueLEDs.set(true);
    }
    else{
      blueLEDs.set(false);

    }
    

    }  
  }
