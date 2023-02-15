// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Elevator;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// // import frc.robot.subsystems.Intake;

// public class intakeCommand extends CommandBase {
//   /** Creates a new moveIntake. */
//   boolean isOnTarget = false;
//   // static Intake m_intake;
//   AnalogInput pressureSensor = new AnalogInput(Constants.Intake.PRESSURE_SENSOR_ID);
//   static intakeCommand instance;

//   PIDController controller = new PIDController(1, 0, 0);
//   public intakeCommand(Intake m_intake) {
//     intakeCommand.m_intake = m_intake;
//     addRequirements(m_intake);
//     instance = new intakeCommand(m_intake);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     isOnTarget = false;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}
//   public void openIntake(){
//     controller.setSetpoint(Constants.Intake.OPEN_POSITION);
//     m_intake.moveMotor(controller.calculate(m_intake.getPosition()));
//     if(controller.atSetpoint()){
//       isOnTarget = true;
//     }
//   }
//   public void closeIntake(){
//     controller.setSetpoint(Constants.Intake.CLOSED_POSITION);
//     if(controller.atSetpoint()){
//       isOnTarget = true;
//     }
//   }

  
//   public void closeWithPressureSensor(){
//     // if(pressureSensor.)
//   }
//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     if(isOnTarget){
//       return true;
//     }
//     return false;
//   }
//   public static intakeCommand getInstance(){
//     if(instance == null){
//       return new intakeCommand(m_intake);
//     }
//     return instance;
//   }
// }
