package frc.robot.commands;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class MoveToPose extends CommandBase {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
  
  private static final int TAG_TO_CHASE = 2;
  private static Transform3d TAG_TO_GOAL;

  boolean finished = false;

  private static final Transform3d CONE_TO_GOAL = new Transform3d(new Translation3d(1.5, 0, 0), new Rotation3d());
  private static final Transform3d CUBE_TO_GOAL = new Transform3d(new Translation3d(1.5, 0, 0), new Rotation3d());
  private static PhotonCamera photonCamera;
  private static Swerve s_Swerve;
  private static Pose2d poseProvider;

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  static MoveToPose instance;
  
  private PhotonTrackedTarget lastTarget;

  enum dropOffPoint{
    TO_LEFT,
    TO_CENTER,
    TO_RIGHT
  }


  static Pose2d goalPose;

  public MoveToPose(
        PhotonCamera photonCamera, 
        Swerve s_Swerve,
        Pose2d poseProvider,
        Pose2d goalPose) {
    this.goalPose = goalPose;
          
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
    var robotPose = poseProvider;
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose2d = poseProvider;
    var robotPose = 
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0, 
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
    
          xController.setGoal(goalPose.getX());
          yController.setGoal(goalPose.getY());
          omegaController.setGoal(goalPose.getRotation().getRadians());
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

  public static MoveToPose getInstance(){
    if(instance == null){
      return new MoveToPose(photonCamera, s_Swerve, poseProvider, goalPose);
    }
    return instance;
  }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.stop();
  }

  public boolean isFinished(){
    return omegaController.atGoal() && yController.atGoal() && xController.atGoal();
  }

}