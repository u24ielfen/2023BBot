package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    
    private Swerve s_Swerve;
    private XboxController controller;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, XboxController controller, boolean fieldRelative) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.fieldRelative = fieldRelative;
        this.controller = controller;
    }

    @Override
    public void execute() {
        if(fieldRelative == true){
            SmartDashboard.putBoolean("FieldRelative", true);
        }else{
            SmartDashboard.putBoolean("FieldRelative", false);
        }
        double yAxis = -controller.getLeftY();
        double xAxis = controller.getLeftX();
        double rAxis = -controller.getRightX();
        
        yAxis = Math.copySign(yAxis*yAxis, yAxis);
        xAxis = Math.copySign(xAxis*xAxis, xAxis);
        rAxis = Math.copySign(rAxis*rAxis, rAxis);
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;
        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        
        // s_Swerve.drive(translation, rotation, fieldRelative);
        s_Swerve.drive(translation, rotation, controller.getLeftBumper());
    }
}
