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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class MoveToTag extends CommandBase {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = 
      new TrapezoidProfile.Constraints(8, 8);
  
  private static final int TAG_TO_CHASE = 2;
  private static final Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(180.));

  private final PhotonCamera photonCamera;
  private final Swerve s_Swerve;
  private final Pose2d poseProvider;

  private final ProfiledPIDController xController = new ProfiledPIDController(4, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(4, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRATINTS);

  private Pose2d goalPose;
  private PhotonTrackedTarget lastTarget;

  public MoveToTag(
        PhotonCamera photonCamera, 
        Swerve s_Swerve,
        Pose2d poseProvider) {
    this.photonCamera = photonCamera;
    this.s_Swerve = s_Swerve;
    this.poseProvider = poseProvider;

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-1, 1);

    addRequirements(s_Swerve);
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
      SmartDashboard.putBoolean("HAs", true);
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
            
            var cameraPose = robotPose.transformBy(Constants.CAMERA_TO_ROBOT.inverse());
            Pose2d targetPose = cameraPose.transformBy(transform);
            
            goalPose = targetPose.transformBy(TAG_TO_GOAL);
        }

        if (null != goalPose) {
          xController.setGoal(goalPose.getX());
          yController.setGoal(goalPose.getY());
          omegaController.setGoal(goalPose.getRotation().getRadians());
        
            
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
        
        SmartDashboard.putNumber("xMove", xSpeed);
        SmartDashboard.putNumber("omeeega", xSpeed);
        SmartDashboard.putNumber("yMove", ySpeed);
        Translation2d movement = new Translation2d(xSpeed, -ySpeed);
        s_Swerve.drive(
            movement, -omegaSpeed, false);
        
      }
      else{
        
        s_Swerve.drive(new Translation2d(0, 0), 0, false);
        SmartDashboard.putBoolean("HAs", false);
      }
            
      }
    } 
  }
  @Override
  public void end(boolean interrupted) {
    // Swerve.stop();
  }

}