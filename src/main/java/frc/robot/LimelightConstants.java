// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;

/** Add your docs here. */
public class LimelightConstants {
    public static final double limelightMountAngle = -2.31;
    public static final double limelightMountHeight = 0.3575;



    public enum ledMode{
        PIPELINE_DEFUALT(0),
        FORCE_ON(1),
        FORCE_BLINK(2),
        FORCE_OFF(3);

        private final int value;

        public int getValue(){
            return value;
        }

        ledMode(int value){
            this.value = value;
        }
    }

    enum camMode{
        VISION_PROCESSOR,
        DRIVER_CAM
    }

    enum stream{
        STANDARD,
        PIP_MAIN,
        PIP_SECONDARY
    }

    enum snapshot{
        RESET,
        TAKE_SNAPSHOT
    }
}
