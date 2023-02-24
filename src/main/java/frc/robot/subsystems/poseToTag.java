// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import javax.swing.text.StyleContext.SmallAttributeSet;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class poseToTag extends SubsystemBase {
  /** Creates a new poseToTag. */
  static poseToTag instance;
  static Supplier<Pose2d> poseProvider;
  static PhotonCamera photonCamera;
  
  Transform3d TAG_TO_GOAL = new Transform3d(new Translation3d(), new Rotation3d());
  Pose2d goalPose;

  PhotonTrackedTarget lastTarget;
  public poseToTag(Supplier<Pose2d> poseProvider, PhotonCamera photonCamera) {
    this.poseProvider = poseProvider;
    this.photonCamera = photonCamera;
  }

  
  @Override
  public void periodic() {
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
        
        SmartDashboard.putNumber("Distance From Tag X", goalPose.getX());
        SmartDashboard.putNumber("Distance From Tag Y", goalPose.getY());
    // This method will be called once per scheduler run
  }
  }
  }
  public Pose2d getPose(){
    return goalPose;
    
  }
  public static poseToTag getInstance(){
    if(instance == null){
      return new poseToTag(poseProvider, photonCamera);
    }
    return instance;
  }

  public PhotonCamera getPhotonCam(){
    return photonCamera;
  }
}
    
