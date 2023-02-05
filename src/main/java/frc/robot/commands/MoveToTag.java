package frc.robot.commands;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class MoveToTag extends CommandBase {
  
   TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
   TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
   TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = 
      new TrapezoidProfile.Constraints(8, 8);
  
   int TAG_TO_CHASE = 2;
   Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180.));
   Transform2d LIMELIGHT_TO_GOAL = new Transform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(180));
   Transform2d CONE_TO_GOAL= new Transform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(180));
   static PhotonCamera photonCamera;
   static Swerve s_Swerve;
   static Pose2d poseProvider;

  ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
  ProfiledPIDController yController = new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
  ProfiledPIDController omegaController = new ProfiledPIDController(0.5, 0, 0, OMEGA_CONSTRATINTS);

  private Pose2d goalPose;
  private PhotonTrackedTarget lastTarget;
  int pipeline = 0;

  Pair<Pose2d, Double> pair;

  public MoveToTag(
        PhotonCamera photonCamera, 
        Swerve s_Swerve,
        Pose2d poseProvider) {
    this.photonCamera = photonCamera;
    this.s_Swerve = s_Swerve;
    this.poseProvider = poseProvider;

    xController.setTolerance(1);
    yController.setTolerance(1);
    omegaController.setTolerance(Units.degreesToRadians(5));
    omegaController.enableContinuousInput(-1, 1);

    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {
    goalPose = null;
    lastTarget = null;
    omegaController.reset(poseProvider.getRotation().getRadians());
    xController.reset(poseProvider.getX());
    yController.reset(poseProvider.getY());
  }

  @Override
  public void execute() {
    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
          .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        if (!target.equals(lastTarget)) {
          lastTarget = target;

          var camToTarget = target.getBestCameraToTarget();
          var transform = new Transform2d(
            camToTarget.getTranslation().toTranslation2d(),
            camToTarget.getRotation().toRotation2d().minus(Rotation2d.fromDegrees(90)));
            
            var cameraPose = poseProvider.transformBy(Constants.CAMERA_TO_ROBOT.inverse());
            Pose2d targetPose = cameraPose.transformBy(transform);
            
            goalPose = targetPose.transformBy(TAG_TO_GOAL);
        }

        if (null != goalPose) {
          xController.setGoal(goalPose.getX());
          yController.setGoal(goalPose.getY());
          omegaController.setGoal(goalPose.getRotation().getRadians());
        }
      }
    }
    
    var xSpeed = xController.calculate(poseProvider.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }
    
    var ySpeed = yController.calculate(poseProvider.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = omegaController.calculate(poseProvider.getRotation().getRadians());
    if (omegaController.atGoal()) {
      omegaSpeed = 0;
    }
    Translation2d driving = new Translation2d(ySpeed, -ySpeed);
    s_Swerve.drive(
      driving, omegaSpeed, false);
  }

  @Override
  public void end(boolean interrupted) {
    // Swerve.stop();
  }

  public int getTarget(){
    if(hasTarget()){
      return photonCamera.getLatestResult().getBestTarget().getFiducialId();
    }
    else{
      return 0;
    }
  }
  
  public void changePipeline(){ 
    if(pipeline == 0){
      photonCamera.setPipelineIndex(1);
      pipeline = 1;
    }
    else if(pipeline == 1){
      photonCamera.setPipelineIndex(2);
      pipeline = 2;
    }
    
    else if(pipeline == 2){
      photonCamera.setPipelineIndex(0);
      pipeline = 0;
    }
  }
  public boolean hasTarget(){
    return photonCamera.getLatestResult().hasTargets();
  }

  public static MoveToTag getInstance(){
    return new MoveToTag(photonCamera, s_Swerve, poseProvider);
  }

}