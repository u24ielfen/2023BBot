package frc.robot.commands;

import java.lang.constant.Constable;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class MoveToTag extends CommandBase {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
  
  private static final int TAG_TO_CHASE = 2;
  private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(1.5, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));

  private static final Transform3d CONE_TO_GOAL = new Transform3d(new Translation3d(1.5, 0, 0), new Rotation3d());
  private static final Transform3d CUBE_TO_GOAL = new Transform3d(new Translation3d(1.5, 0, 0), new Rotation3d());
  private final PhotonCamera photonCamera;
  private final Swerve s_Swerve;
  private final Supplier<Pose2d> poseProvider;

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  private PhotonTrackedTarget lastTarget;

  public MoveToTag(
        PhotonCamera photonCamera, 
        Swerve s_Swerve,
        Supplier<Pose2d> poseProvider) {
    this.photonCamera = photonCamera;
    this.s_Swerve = s_Swerve;
    this.poseProvider = poseProvider;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();
    var robotPose = 
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0, 
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
    
    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      if(photonCamera.getPipelineIndex() == Constants.VisionConstants.aprilTagPipeline){

        var targetOpt = photonRes.getTargets().stream()
        .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
        .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
        .findFirst();
        if (targetOpt.isPresent()) {
          var target = targetOpt.get();
          lastTarget = target;
          
          var cameraPose = robotPose.transformBy(Constants.APRILTAG_CAMERA_TO_ROBOT);
          
          var camToTarget = target.getBestCameraToTarget();
          var targetPose = cameraPose.transformBy(camToTarget);
          
          var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
          
          xController.setGoal(goalPose.getX());
          yController.setGoal(goalPose.getY());
          omegaController.setGoal(goalPose.getRotation().getRadians());
        }
      }
      else if(photonCamera.getPipelineIndex() == Constants.VisionConstants.conePipeline){

        var targetOpt = photonRes.getTargets().stream().findFirst();
        // .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
        // .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
        // .findFirst();
        if (targetOpt.isPresent()) {
          var target = targetOpt.get();
          lastTarget = target;
          
          var cameraPose = robotPose.transformBy(Constants.CONE_CUBE_CAMERA_TO_ROBOT);
          
          var camToTarget = target.getBestCameraToTarget();
          var targetPose = cameraPose.transformBy(camToTarget);
          
          var goalPose = targetPose.transformBy(CONE_TO_GOAL).toPose2d();
          
          xController.setGoal(goalPose.getX());
          yController.setGoal(goalPose.getY());
          omegaController.setGoal(goalPose.getRotation().getRadians());
        }
      }
        else if(photonCamera.getPipelineIndex() == Constants.VisionConstants.cubePipeline){

          var targetOpt = photonRes.getTargets().stream().findFirst();
          // .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
          // .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
          // .findFirst();
          if (targetOpt.isPresent()) {
            var target = targetOpt.get();
            lastTarget = target;
            
            var cameraPose = robotPose.transformBy(Constants.CONE_CUBE_CAMERA_TO_ROBOT);
            
            var camToTarget = target.getBestCameraToTarget();
            var targetPose = cameraPose.transformBy(camToTarget);
            
            var goalPose = targetPose.transformBy(CUBE_TO_GOAL).toPose2d();
            
            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());
        }
      }
    }
    
    if (lastTarget == null) {
      s_Swerve.stop();
    } else {
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }

      Translation2d movement = new Translation2d(xSpeed, ySpeed);

      s_Swerve.drive(movement, omegaSpeed, true);
    }
  }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.stop();
  }

}