package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class BalanceOnRamp extends CommandBase {
	
	private final AHRS gyro;
	Swerve s_Swerve;
	private double currentAngle =0;
	private double lastAngle = 0;
	private boolean hasPassedHighPoint;
	private final double highPoint = Math.toRadians(16);
	private final double minAngleChangeToStop = Math.toRadians(0.25);
	private final double speed = 0.25;
	
	public BalanceOnRamp(Swerve s_Swerve){
		this.s_Swerve = s_Swerve;
		this.gyro = s_Swerve.getAhrs();
		addRequirements(s_Swerve);
	}
	
	@Override
	public void execute() {
		lastAngle = currentAngle;
		currentAngle = Math.abs(gyro.getRoll());
		s_Swerve.moveByChassisSpeeds(speed * -Math.signum(gyro.getRoll()), 0,0,0);
		
		if (currentAngle > highPoint){hasPassedHighPoint = true;}
	}
	
	@Override
	public boolean isFinished() {
		if(currentAngle - lastAngle <= -1 * minAngleChangeToStop && hasPassedHighPoint){
			SmartDashboard.putBoolean("on",false);
			return true;
		}
		return false;
	}
	
	@Override
	public void end(boolean interrupted) {
		s_Swerve.stop();
		hasPassedHighPoint = false;
	}
}