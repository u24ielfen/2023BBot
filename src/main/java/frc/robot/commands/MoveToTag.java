package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class MoveToTag extends CommandBase {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = 
      new TrapezoidProfile.Constraints(8, 8);
  
  private static final int TAG_TO_CHASE = 8;
  private static final Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(180.0));

  private final PhotonCamera photonCamera;
  private final Swerve swerve;
  private final Pose2d poseProvider;

  private final ProfiledPIDController xController = new ProfiledPIDController(4, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(4, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRATINTS);

  private Pose2d goalPose;
  private PhotonTrackedTarget lastTarget;

  public MoveToTag(
        PhotonCamera photonCamera, 
        Swerve swerve,
        Pose2d poseProvider) {
    this.photonCamera = photonCamera;
    this.swerve = swerve;
    this.poseProvider = poseProvider;

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-1, 1);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    goalPose = null;
    lastTarget = null;
    var robotPose = poseProvider;
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose = poseProvider;
    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
          .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        if (!target.equals(lastTarget)) {
          // This is new target data, so recalculate the goal
          lastTarget = target;

          // Get the transformation from the camera to the tag (in 2d)
          var camToTarget = target.getBestCameraToTarget();
          var transform = new Transform2d(
            camToTarget.getTranslation().toTranslation2d(),
            camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));
            
            // Transform the robot's pose to find the tag's pose
            var cameraPose = robotPose.transformBy(Constants.CAMERA_TO_ROBOT.inverse());
            Pose2d targetPose = cameraPose.transformBy(transform);
            
            // Transform the tag's pose to set our goal
            goalPose = targetPose.transformBy(TAG_TO_GOAL);
        }

        if (null != goalPose) {
          // Drive
          xController.setGoal(goalPose.getX());
          yController.setGoal(goalPose.getY());
          omegaController.setGoal(goalPose.getRotation().getRadians());
        }
      }
    }
    
    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }
    
    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
    if (omegaController.atGoal()) {
      omegaSpeed = 0;
    }
    Translation2d movement = new Translation2d(xSpeed, ySpeed);
    Swerve.drive(
        movement, omegaSpeed, true);
        
  }
  @Override
  public void end(boolean interrupted) {
    // Swerve.stop();
  }

}