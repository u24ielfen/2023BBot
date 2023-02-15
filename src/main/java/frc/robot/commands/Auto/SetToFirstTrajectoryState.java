package frc.robot.commands.Auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.Swerve;
// import frc.robot.subsystems.poseEstimator;

public class SetToFirstTrajectoryState extends ScheduleCommand {

    private final static double EPSILON = 0.00001;
    private final static double MODULE_ANGLE_TOLERANCE = Units.degreesToRadians(4);
    // poseEstimator estimator = poseEstimator.getInstance();
    PathPlannerTrajectory path;
    Swerve s_Swerve = new Swerve();

    public SetToFirstTrajectoryState(PathPlannerTrajectory trajectory) {
        this.path = trajectory;
    }

    @Override
    public void initialize() {
        // estimator.setCurrentPose(path.getInitialHolonomicPose());
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("pre auto command", true);
        PathPlannerTrajectory.PathPlannerState state = path.getInitialState();

        double leftwardSpeed =
                state.poseMeters.getRotation().getSin() * state.velocityMetersPerSecond;
        double forwardSpeed =
                state.poseMeters.getRotation().getCos() * state.velocityMetersPerSecond;
        ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed * EPSILON, leftwardSpeed * EPSILON, state.holonomicAngularVelocityRadPerSec * EPSILON);
        s_Swerve.moveByChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
        // boolean allInAngle = true;
        // for (SwerveChassis.Module module : SwerveChassis.Module.values()) {
        //     allInAngle &= s_Swerve.moduleIsAtAngle(module, MODULE_ANGLE_TOLERANCE);
        // }
        // return allInAngle;
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.stop();
        SmartDashboard.putBoolean("pre auto command", false);
    }
}