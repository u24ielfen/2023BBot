package frc.robot.commands.Auto;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.poseEstimator;
import frc.robot.Constants;
import frc.robot.commands.Elevator.armCommands;

import java.util.HashMap;


public class PathFollowerBuilder  {
	static Swerve s_Swerve = new Swerve(); 
	static poseEstimator estimator = poseEstimator.getInstance();

	//ARM COMMANDS
	static armCommands c_ArmToTop = new armCommands("Arm To Top");
	static armCommands c_ArmToMid = new armCommands("Arm To Mid");
	static armCommands c_ArmToBottom = new armCommands("Arm To Bottom");
	
	static armCommands c_I_ArmToTop = new armCommands("Inverted Arm To Top");
	static armCommands c_I_ArmToMid = new armCommands("Inverted Arm To Mid");
	static armCommands c_I_ArmToBottom = new armCommands("Inverted Arm To Bottom");

	private static final HashMap<String, Command> eventMap = new HashMap<>();
	public static final double DEADLINE_TIME_FOR_PRE_AUTO_COMMAND = 1;
	PPSwerveControllerCommand builder;
	SwerveAutoBuilder builder1;
	PathPlannerTrajectory path;

	
	static PathFollowerBuilder instance;
	
	public PathFollowerBuilder() {
		// eventMap.put("place", new DelayCommand( 2, "oneThing"));
		// eventMap.put("toTop", new armCommands("Arm To Top"));
		builder1 = new SwerveAutoBuilder(() -> estimator.getCurrentPose(), (Pose2d pose) -> estimator.resetChassisPose(),
			Constants.Swerve.kinematics, new PIDConstants(1, 0, 0), new PIDConstants(1, 0, 0),
			s_Swerve::setModuleStates, eventMap, s_Swerve);
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
		switch (pathName) {
			case "Nothing":
				break;
			case "Another Thing":
				// addEventMap("Pick Up", );

			break;
			default:
				break;
		}
        path = PathPlanner.loadPath(
                pathName, new PathConstraints(
                        1,
                        1
                ));
				return builder1.fullAuto(path);
			}
			
			public static PathPlannerTrajectory getPathPlannerTrajectory(String path) {
		return PathPlanner.loadPath(path, new PathConstraints(1, 2));
	}
	
}