package frc.robot.commands.Auto;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.poseEstimator;
import frc.robot.Constants;
// import frc.robot.subsystems.Intake;
import frc.robot.commands.Elevator.ArmController;

import java.util.HashMap;


public class PathFollowerBuilder  {
	static Swerve s_Swerve = new Swerve(); 
	static poseEstimator estimator = poseEstimator.getInstance();
	static ArmController c_ArmController = new ArmController();
	private static final HashMap<String, Command> eventMap = new HashMap<>();
	public static final double DEADLINE_TIME_FOR_PRE_AUTO_COMMAND = 1;
	SwerveAutoBuilder builder;
	SendableChooser chooser = new SendableChooser<>();
	
	static {
       eventMap.put("place", new DelayCommand( 2, "oneThing"));
	   eventMap.put("toTop", new DelayCommand(3, "wow").alongWith(c_ArmController.setArmToTop()));		
	}
	
	private static PathFollowerBuilder instance;
	
	private PathFollowerBuilder() {
		builder = new SwerveAutoBuilder(() -> estimator.getCurrentPose(),(Pose2d pose) -> estimator.resetChassisPose(), new PIDConstants(1, 0, 0), new PIDConstants(1, 0, 0), null, eventMap, s_Swerve);
	}
	
	public void addEventMap(String word, Command cmd){
		eventMap.put(word, cmd);
	}

	public static PathFollowerBuilder getInstance() {
		if (instance == null) {
			instance = new PathFollowerBuilder();
		}
		return instance;
	}
	
	public CommandBase followPath(String pathName) {
		
		PathPlannerTrajectory path;
        path = PathPlanner.loadPath(
                pathName, new PathConstraints(
                        1,
                        1
                ));
		
		
        return builder.fullAuto(path);
	}
	
	public static PathPlannerTrajectory getPathPlannerTrajectory(String path) {
		return PathPlanner.loadPath(path, new PathConstraints(1, 2));
	}
	
}