// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.PipedInputStream;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants;


public class Vision extends SubsystemBase {
  /** Creates a new Limelight. */

  NetworkTable table;
  NetworkTableEntry targetY;
  NetworkTableEntry targetX;
  NetworkTableEntry targetArea;
  PhotonCamera coneCubeCamera;

  public Vision(PhotonCamera coneCubeCamera) {
      table = NetworkTableInstance.getDefault().getTable("limelight");
      targetY = table.getEntry("ty");
      targetX = table.getEntry("tx");
      targetArea = table.getEntry("ta");
      this.coneCubeCamera = coneCubeCamera;
  }

  public PhotonCamera getCamera(){
    return coneCubeCamera;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = coneCubeCamera.getLatestResult();
    SmartDashboard.putNumber("Cone Distance", cubeConeDistance(0.0, result));
    heightToWidthRation(result.getBestTarget());
    checkConeOrientation(cubeConeDistance(0,result), result);
  }

  //Calculates Pitch to bottom edge
  public double absoluteBottomeEdgePitch(PhotonPipelineResult result){
    if(result != null){

      List<TargetCorner> corners = result.getBestTarget().getDetectedCorners();
      double lowestPoint[] = {0,0};
    for(TargetCorner corner : corners){
      if(corner.y >= lowestPoint[0]){
        lowestPoint[1] = lowestPoint[0];
        lowestPoint[0] = corner.y;
      }
      else if(corner.y >= lowestPoint[1]){
        lowestPoint[1] = corner.y;
      }
    }

    double y = lowestPoint[0] + lowestPoint[1];
    
    
    y/=2;
    
    y-=120;
    y*=-1;
    
    SmartDashboard.putNumber("Y", y);
    y = y/120 * 0.3727;
    return Math.asin(y) * 180 / Math.PI;
  }
  else{
    return 0;
  }
  }

  public double heightToWidthRation(PhotonTrackedTarget target){
    if(target != null){

      List<TargetCorner> corners = target.getDetectedCorners();
      
          double wdx = Math.pow(corners.get(1).x - corners.get(2).x, 2);
          double wdy = Math.pow(corners.get(1).y - corners.get(2).y, 2);
          double hdx = Math.pow(corners.get(1).x - corners.get(0).x, 2);
          double hdy = Math.pow(corners.get(1).y - corners.get(0).y, 2);
      
          double xarr[] = {0,0,0,0};
          double yarr[] = {0,0,0,0};
          for(int i = 0; i < 4; i++){
            xarr[i] = corners.get(i).x;
            yarr[i] = corners.get(i).y;
      
          }
          SmartDashboard.putNumberArray("Xarr", xarr);
          SmartDashboard.putNumberArray("Yarr", yarr);
      
          double width = Math.sqrt(wdx + wdy);
          double height = Math.sqrt(hdx + hdy);
      
          SmartDashboard.putNumber("WtoH", width/height);
          return height/width;
    }
    else{
      return 0;
    }
  }

  public double cubeConeDistance(double targetHeight, PhotonPipelineResult result){
    if(result.hasTargets() == false){
      return 0;
    }
    SmartDashboard.putNumber("Weird Calculated thing", absoluteBottomeEdgePitch(result));
    
    PhotonTrackedTarget target = result.getBestTarget();


    //Get the vertical angle offset from limelight
    double verticalOffsetAngle = absoluteBottomeEdgePitch(result);


    //get the angle to goal in degrees
    double angleToGoalDegrees = VisionConstants.ConeCubeCamera.mountingAngle + verticalOffsetAngle;

    //convert angle to radians
    double angleToGoalRadians = angleToGoalDegrees * (3.1415926/180);

    //calculate distance to target
    double dist = (targetHeight-VisionConstants.ConeCubeCamera.mountHeight)/Math.tan(angleToGoalRadians);
    return dist;
  }


  public double limeGetDistance(double targetHeight){
      //Get the vertical angle offset from limelight
      double verticalOffsetAngle = targetY.getDouble(0.0);

      //get the angle to goal in degrees
      double angleToGoalDegrees = VisionConstants.Limelight.mountingAngle + verticalOffsetAngle;

      //convert angle to radians
      double angleToGoalRadians = angleToGoalDegrees * (3.1415926/180);

      //calculate distance to target
      double dist = (targetHeight-VisionConstants.Limelight.limeLightHeight)/Math.tan(angleToGoalRadians);
      return dist;
  }

  //Calculate theoretical area of a cone based on distance --> need to tune constant for this
  public void checkConeOrientation(double distance, PhotonPipelineResult result){
    if(result != null){

      PhotonTrackedTarget target = result.getBestTarget();
      if(target != null){

        double vertConeError = Math.abs(VisionConstants.ConeCubeCamera.coneLongAreaConstant/Math.pow(distance, 2) - target.getArea());
        double baseConeError = Math.abs(VisionConstants.ConeCubeCamera.coneShortAreaConstant/Math.pow(distance, 2) - target.getArea());
        
        if(vertConeError <= baseConeError){
          if(Math.abs(target.getPitch() - absoluteBottomeEdgePitch(result)) < 1.5){
          SmartDashboard.putString("Cone orientation", "Horizontal");
      }else{
        SmartDashboard.putString("Cone orientation", "Upright");
      }
    }else{
      SmartDashboard.putString("Cone orientation", "Nose or base first");
    }
  }

  }

  }


  //get the x angle from the limelight
  public double limeGetAngleX(){
      return targetX.getDouble(0.0);
  }



  //Allows user to swap between on, off, and blinking LEDS
  public void limeSetLedMode(int ledMode){
    table.getEntry("ledMode").setNumber(1);
  }

  //Toggle between vision processing and driver cam
  public void limeSetCamMode(int camMode){
    table.getEntry("camMode").setNumber(camMode);
  }

  //Toggle between different stream layouts
  public void limeSetStreamMode(int streamMode){
    table.getEntry("stream").setNumber(streamMode);
  }

  //Take snapshot and reset snapshot mode
  public void limeSnapshot(int snapMode){
    table.getEntry("snapshot").setNumber(snapMode);
  }

  //Change pipelines
  public void limeSetPipeline(int pipeline){
    table.getEntry("pipeline").setNumber(pipeline);
  }


  public void photonSetPipeline(PhotonCamera cam, int pipeline){
    cam.setPipelineIndex(pipeline);
  }


}
