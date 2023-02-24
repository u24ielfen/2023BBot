package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;


import org.photonvision.PhotonCamera;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public static SwerveModule[] mSwerveMods;
    public static AHRS gyro;
    public static Field2d field2d;
    static Swerve instance = new Swerve();
    // static poseEstimator estimator = new poseEstimator(null, instance);
    public static AprilTagFieldLayout layout;
    
    public Swerve() {
        gyro = new AHRS(SPI.Port.kMXP);
        zeroGyro();
        field2d = new Field2d();
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        
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
    public void moveByChassisSpeeds(ChassisSpeeds speed){
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speed, getGyro()));
        setModuleStates(swerveModuleStates);

    }

    public void resetChassisPose(Pose2d pose){
        // estimator.setCurrentPose(pose);

    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredAutoState(desiredStates[mod.moduleNumber]);
        }
    }    


    public boolean isAtState(){
        return false;
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

    public Rotation2d getRoll(){
        return Rotation2d.fromDegrees(gyro.getRoll());
    }
    @Override
    public void periodic(){
        // if(SmartDashboard.getNumber("Distance", 0) > 0.5){
        //     drive(new Translation2d(0, 0.1), 0, false);
        // }
        // else{
        //     drive(new Translation2d(0, 0), 0, false);
        // }
        SmartDashboard.putNumber("Gyro Pitch", getPitch().getDegrees());
        // field2d.setRobotPose(getEstimatedPose());
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

    public AHRS getAhrs(){
        return gyro;
    }
    public void stop(){        
        for(SwerveModule mod : mSwerveMods){
            mod.stop();
        }
    }
    
  public Command createCommandForTrajectory(Trajectory trajectory, Supplier<Pose2d> poseSupplier) {
    
      return null;
  }

public double getCurrentTotalVelocity() {
    return 0;
}

public Rotation2d getHeading() {
    return null;
}

public Rotation2d getYaw() {
    return null;
}

public static Swerve getInstance(){
    if(instance == null){
        return new Swerve();
    }
    return instance;
}

public void moveByChassisSpeeds(double forwardSpeed, double leftwardSpeed, double angSpeed, double currentAng) {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardSpeed,
            leftwardSpeed,
            angSpeed,
            Rotation2d.fromDegrees(Math.toDegrees(currentAng)));
    SwerveModuleState[] states = Constants.Swerve.kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
    setModuleStates(states);
}


}