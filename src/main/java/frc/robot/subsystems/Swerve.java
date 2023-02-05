package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.xml.validation.SchemaFactory;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Swerve extends SubsystemBase {
    public static SwerveModule[] mSwerveMods;
    public static AHRS gyro;
    public static Field2d field2d;
    
    //Odometry & Pose Estimation
    public SwerveDriveOdometry swerveOdometry;
    public static SwerveDrivePoseEstimator m_PoseEstimator;
    public static PhotonPoseEstimator aprilTagPoseEstimator;
    
    public static AprilTagFieldLayout layout;
    
    
    public Swerve() {
        
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        gyro = new AHRS(Port.kMXP);
        zeroGyro();
        field2d = new Field2d();
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        //Odometry
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.kinematics, getGyro(), getModulePositions());
        //Pose Estimation
        m_PoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.kinematics, getGyro(), getModulePositions(), new Pose2d(), Constants.VisionConstants.STATE_STD_DEVS, Constants.VisionConstants.VISION_MEASUREMENT_STD_DEVS);
        
        
        //AprilTag pose estimator
        aprilTagPoseEstimator = new PhotonPoseEstimator(layout, Constants.VisionConstants.APRILTAG_POSE_STRATEGY, Constants.VisionConstants.APRILTAG_CAM, Constants.VisionConstants.APRILTAG_CAM_POS);
        
        List<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(Constants.VisionConstants.APRILTAG_CAM, Constants.VisionConstants.APRILTAG_CAM_POS));
        

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getY(), 
                                    translation.getX(), 
                                    rotation, 
                                    getGyro()
                                )
                                : new ChassisSpeeds(
                                    translation.getY(), 
                                    translation.getX(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        SmartDashboard.putNumber("Gyrothingiesies", getGyro().getDegrees());
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
        }
    }   
    
    public void resetWithController(){
        for(SwerveModule mod: mSwerveMods){
            mod.resetWithController();
        }
    }
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredAutoState(desiredStates[mod.moduleNumber]);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose(){
        return m_PoseEstimator.getEstimatedPosition();
    }

    public void resetToVision() {

        //TODO this is probably wrong (Optional<Pair<Pose2d, Double>>)
        Optional<EstimatedRobotPose> aprilTagEstimation = getApriltagEstimatedPose(getEstimatedPose());
        if (aprilTagEstimation.isPresent()) {
            swerveOdometry.resetPosition(getGyro(), getModulePositions(), aprilTagEstimation.get().estimatedPose.toPose2d());
            m_PoseEstimator.resetPosition(getGyro(), getModulePositions(), aprilTagEstimation.get().estimatedPose.toPose2d());
        }
    }
    
    public Optional<EstimatedRobotPose> getApriltagEstimatedPose(Pose2d prevEstimatedRobotPose) {
        aprilTagPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return aprilTagPoseEstimator.update();
    }
    
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getGyro(), getModulePositions(), pose);
        m_PoseEstimator.resetPosition(getGyro(), getModulePositions(), pose);
    }
    public Command resetFieldPos(){
        return new SequentialCommandGroup(new InstantCommand(() -> zeroGyro()).andThen(new InstantCommand(
            () -> swerveOdometry.resetPosition(getGyro(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d())), this
        )));
    }
    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro(){
        gyro.reset();
    }

    public Rotation2d getGyro() {
        return Rotation2d.fromDegrees(360- gyro.getYaw());
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyro(), getModulePositions());
        m_PoseEstimator.update(getGyro(), getModulePositions());
        double currentTime = Timer.getFPGATimestamp();

        aprilTagPoseEstimator.setReferencePose(getEstimatedPose());
        Optional<EstimatedRobotPose> result = getApriltagEstimatedPose(m_PoseEstimator.getEstimatedPosition());

        if(result.isPresent()){
            EstimatedRobotPose camPose = result.get();
            m_PoseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
            //TODO: Add field2d & else statement
            field2d.getObject("AprilTag").setPose(camPose.estimatedPose.toPose2d());
        }

        SmartDashboard.putNumber("Gyro Pitch", getPitch().getDegrees());
        field2d.setRobotPose(getEstimatedPose());
        SmartDashboard.putNumber("Gyro", getGyro().getDegrees());
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }
    public Rotation2d getPitch(){
        return Rotation2d.fromDegrees(gyro.getPitch());
    }
    public void setModuleRoation(Rotation2d rotation){
        for(SwerveModule mod: mSwerveMods){
            mod.setDesiredState(new SwerveModuleState(0, rotation));
        }
    }
    public SwerveModule[] getModules(){
        return mSwerveMods;
    }
  public SwerveControllerCommand getCommand(Trajectory trajectory){
    var thetaController = new ProfiledPIDController(
      Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
    return new SwerveControllerCommand(
          trajectory,
          this::getPose,
          Constants.Swerve.kinematics,
          new PIDController(Constants.AutoConstants.kPXController, 0, 0),
          new PIDController(Constants.AutoConstants.kPYController, 0, 0),
          thetaController,
          this::setModuleStates,
          this);
  }
  
}