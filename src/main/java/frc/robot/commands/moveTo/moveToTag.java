// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.moveTo;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.MoveToPose;
import frc.robot.subsystems.poseEstimator;

public class moveToTag extends CommandBase {
  /** Creates a new moveToTag. */
  MoveToPose c_MoveToPose = MoveToPose.getInstance();

  static moveToTag instance;
  static PhotonCamera photonCamera;
  Transform3d TAG_TO_GOAL = new Transform3d(new Translation3d(), new Rotation3d());
  static Supplier<Pose2d> poseProvider;
  Pose2d goalPose;

  PhotonTrackedTarget lastTarget;

  public moveToTag(PhotonCamera photonCamera, Supplier<Pose2d> poseProvider ) {
    this.photonCamera = photonCamera;
    this.poseProvider = poseProvider;
    
    // Use addRequirements() here to declare subsystem dependencies.
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
      var targetOpt = photonRes.getTargets().stream()
          // .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
          .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        lastTarget = target;
        
        var cameraPose = robotPose.transformBy(Constants.APRILTAG_CAMERA_TO_ROBOT.inverse());

        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);
        goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
        
      }
    }
  }
  public Pose2d getGoalPose(){
    return goalPose;
  }
  public static moveToTag getInstance(){
    if(instance == null){
      return new moveToTag(photonCamera, poseProvider);
    }
    return instance;
  }

  @Override
  public boolean isFinished() {
    return c_MoveToPose.isFinished();
  }
}
