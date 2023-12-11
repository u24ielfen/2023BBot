package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private BooleanSupplier fieldRelative;
    
    private Swerve s_Swerve;
    private XboxController controller;
    SlewRateLimiter limiter = new SlewRateLimiter(3);
    double maxSpeed = 1;
    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, XboxController controller, BooleanSupplier fieldRelative) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.fieldRelative = fieldRelative;
        this.controller = controller;
    }
   
    @Override
    public void execute() {
        boolean isFieldRelative = fieldRelative.getAsBoolean();
        SmartDashboard.putBoolean("FieldRelative", isFieldRelative);

        double yAxis = -controller.getLeftY();
        double xAxis = -controller.getLeftX();
        double rAxis = -controller.getRightX();

        yAxis = Math.copySign(yAxis * yAxis, Math.signum(yAxis));
        xAxis = Math.copySign(xAxis * xAxis, Math.signum(xAxis));
        rAxis = Math.copySign(rAxis * rAxis, Math.signum(rAxis));
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;
        translation = new Translation2d(xAxis, yAxis);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;

        s_Swerve.drive(translation.times(0.8), rotation * 0.8, isFieldRelative);
    }
}
