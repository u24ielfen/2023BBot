package frc.robot.commands.Auto;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.poseEstimator;
import frc.robot.Constants;
import frc.robot.commands.Elevator;
import frc.robot.subsystems.Intake;

import java.util.HashMap;


public class PathFollowerBuilder extends SwerveAutoBuilder {
	static Swerve s_Swerve = new Swerve(); 
	static poseEstimator estimator = poseEstimator.getInstance():
	private static final HashMap<String, Command> eventMap = new HashMap<>();
	public static final double DEADLINE_TIME_FOR_PRE_AUTO_COMMAND = 1;
	 static Elevator elevator = Elevator.getInstance();
	
	static {
       eventMap.put("place", new DelayCommand( 2, "oneThing"));
		
	}
	
	private static PathFollowerBuilder instance;
	
	private PathFollowerBuilder() {
		super(
				s_Swerve.getGyro(),
				(Pose2d pose) -> estimator.resetChassisPose(pose),
				Constants.Swerve.kinematics,
				new PIDController(1, 0, 0),
				new PIDController(1, 0, 0),
				s_Swerve::setModuleStates,
				eventMap,
				s_Swerve);
	}
	
	
	public static PathFollowerBuilder getInstance() {
		if (instance == null) {
			instance = new PathFollowerBuilder();
		}
		return instance;
	}
	
	
	/**
	 * @return returns the command for the full auto (commands and trajectory included)
	 */
	public CommandBase followPath(String pathName) {
		
		PathPlannerTrajectory path;
        path = PathPlanner.loadPath(
                pathName, new PathConstraints(
                        RobotMap.Swerve.Pegaswerve.MAX_VELOCITY,
                        RobotMap.Swerve.Pegaswerve.MAX_ACCELERATION
                ));
//		switch (RobotMap.robotName) {
//			case pegaSwerve:
//				path = PathPlanner.loadPath(
//						pathName, new PathConstraints(
//								RobotMap.Swerve.Pegaswerve.MAX_VELOCITY,
//								RobotMap.Swerve.Pegaswerve.MAX_ACCELERATION
//						));
//				break;
//			case TobyDetermined:
//				path = PathPlanner.loadPath(
//						pathName, new PathConstraints(
//								RobotMap.Swerve.TobyDetermined.MAX_VELOCITY,
//								RobotMap.Swerve.TobyDetermined.MAX_ACCELERATION
//						));
//				break;
//			default:
//                path = PathPlanner.loadPath(
//                        pathName, new PathConstraints(
//                                RobotMap.Swerve.Pegaswerve.MAX_VELOCITY,
//                                RobotMap.Swerve.Pegaswerve.MAX_ACCELERATION
//                        ));
//				break;
//		}
		
		
		//pathplanner was acting wierd when starting position was not the one defined in the path so i added a reset to the path start
        return fullAuto(path).beforeStarting(new SetToFirstTrajectoryState(path).raceWith(new WaitCommand(DEADLINE_TIME_FOR_PRE_AUTO_COMMAND)));
//        return fullAuto(path);
	}
	
	public static PathPlannerTrajectory getPathPlannerTrajectory(String path) {
		return PathPlanner.loadPath(path, new PathConstraints(RobotMap.Swerve.Pegaswerve.MAX_VELOCITY, RobotMap.Swerve.Pegaswerve.MAX_ACCELERATION));
	}
	
}