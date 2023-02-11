package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.05;

    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT =
        new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d());
    public static final Transform3d APRILTAG_ROBOT_TO_CAMERA = APRILTAG_CAMERA_TO_ROBOT.inverse();

    public static final Transform3d CONE_CUBE_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0.4, 0.3, 0.2), new Rotation3d());

    public static final Transform3d CONE_CUBE_ROBOT_TO_CAMERA = CONE_CUBE_CAMERA_TO_ROBOT.inverse();

    public static final class Swerve {

        /* Drivetrain Constants */
        public static final double trackWidth = 0.517;
        public static final double wheelBase = 0.517;
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (8.14 / 1.0); //6.86:1
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0), //FR
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), //BR
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), //FL
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); //BL
//++, +-, -+, --    FAIL
        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxRobotSpeed = 1; //meters per second
        public static double maxSpeed = maxRobotSpeed;
        public static final double maxAngularVelocity = 1;
        public void changeSpeed(){
            if(maxSpeed == maxRobotSpeed){
                maxSpeed *= 2;
            }
            else{
                maxSpeed = maxRobotSpeed;
            }
        }

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final double angleOffset = 17.929;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 3;
            public static final double angleOffset = -9.6679;
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final double angleOffset = 40.07811;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 11;
            public static final double angleOffset = -38.05664;
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double[] kPXController = {1, 0, 0};
        public static final double[] kPYController = {1, 0, 0};
        public static final double[] kPThetaController = {1, 0, 0};
    
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.kinematics);

      }

      public static final class Elevator{

          public static final int PIVOT_MOTORID = 30;
          public static final int ELEVATOR_GEARING = 100;
          public static final int EXTENDING_MOTORID = 40;
          public static final int PIVOT_GEARING = 100;
          public static final double[] WINCH_PID = {1, 0, 0};
          public static final double[] PIVOT_PID = {1, 0, 0};

          public static final double PIVOT_TICKS_TO_TOP = 100;
          public static final double PIVOT_TICKS_TO_MID = 50;
          public static final double PIVOT_TICKS_TO_BOTTOM = 20;

          public static final double WINCH_TICKS_TO_TOP = 100;
          public static final double WINCH_TICKS_TO_MID = 50;
          public static final double WINCH_TICKS_TO_BOTTOM = 20;

        }

      public static final class Intake{
        public static final int MOTORID = 20;
        public static final double OPEN_POSITION = 100;
        public static final double CLOSED_POSITION = 0;
        public static final double[] PID = {1, 0, 0};
        }
        
    public static class FieldConstants {
        public static final double length = Units.feetToMeters(54);
        public static final double width = Units.feetToMeters(27);
    }

    public static class VisionConstants {
        public static final int conePipeline = 1;
        public static final int cubePipeline = 2;
        public static final int aprilTagPipeline = 3;
        
        public static final Transform3d APRILTAG_CAM_POS = new Transform3d(new Translation3d(0.27, 0.13, 0),
        new Rotation3d(0, -Math.toRadians(20), 0)); // TODO OPI pos
        public static final PhotonCamera APRILTAG_CAM = new PhotonCamera("cam");
        public static final PoseStrategy APRILTAG_POSE_STRATEGY = PoseStrategy.LOWEST_AMBIGUITY;
        /*
        * Increase numbers to trust your model's state estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in
        * meters and radians.
        */
        public static final Matrix<N3, N1> STATE_STD_DEVS = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.10, 0.10, 0.10);

        public static final Matrix<N3, N1> VISION_MEASUREMENT_STD_DEVS = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.90, 0.90, 0.90);
        
        public static final class ConeCubeCamera{
            public static final double mountHeight = 0.3625; 
            public static final double mountingAngle = 4.26;
    
            public static final double coneLongAreaConstant = 10.5165;
            public static final double coneShortAreaConstant = 7.5;
        
        }
            
        public static final class Limelight{
            public static final double limeLightHeight = 0.3575; 
            public static final double mountingAngle = -2.31;
            public static final double verticalConeArea = 2.464;
        
            public static final class ledMode{
            public static final int defaultMode = 0;
            public static final int forceOff = 1;
            public static final int forceBlink = 2;
            public static final int forceOn = 3;
        }
        
        public static final class camMode{
            public static final int visionProccesing = 0;
            public static final int driverCamera = 1;
                }
        
        public static final class stream{
            public static final int standard = 0;
            public static final int main = 1;
            public static final int secondary = 1;
                }
                
        public static final class snapshot{
          public static final int reset = 0;
          public static final int take = 1;
                }
            }

    }
}
