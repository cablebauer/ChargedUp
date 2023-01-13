// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class camera extends SubsystemBase {
  /** Creates a new camera. */
  
  RobotPoseEstimator poseFactory;
  PhotonCamera machineCamera;
  String currentfilter;
  boolean didISucceed;
  //TODO implement a apriltagfield layout. Class seemed to have changed from documentation, supply path via string or path object to proceed
  // do not pass go do not collect 1 million dollars.  
  public camera() {
    //TODO Possibly change this?
    didISucceed = false; 
    //TODO change default to whatever I decide the default filter will be. 
    currentfilter = "Default";
    machineCamera = new PhotonCamera("3468 Main camera"); 
    machineCamera.setDriverMode(!didISucceed);
  }
  public void enabledrivermode (){
    machineCamera.setDriverMode(true);
  }
  public void disabledrivermode(){
    machineCamera.setDriverMode(false);
  }

  @Override
  
  public void periodic() {
    // This method will be called once per scheduler run
    // Comment this out probably but could be useful. Essentially  just displays camera latency 
    SmartDashboard.putNumber("Latency", getResult().getLatencyMillis());
  }
  public PhotonPipelineResult getResult(){
     PhotonPipelineResult results =  machineCamera.getLatestResult();
     // gives photonPipeline results if any valid targets else returns null
     if(results.hasTargets() == false){
      return null;
     }
     else{
      return results;
     }
  }

  public List<PhotonTrackedTarget> gettargets(){
    // returns a list of photon tracked targets with the corropnding information regarding them
    PhotonPipelineResult raw = getResult();
    if(raw != null)
    {return raw.getTargets();}
    else{ 
      return null;}
    
    // For the uninformed this still requires further refinement to acess said information 
  }
  public PhotonTrackedTarget getBesttarget(){
    PhotonPipelineResult raw = getResult();
    if(raw != null){
      return raw.getBestTarget();
    }
    else {return null;}


    }

  public void changeFilter(String filter){
    filter = filter.toLowerCase();
    if(filter == "cube" || filter ==  "cubes" ){
      /* TODO Implement the swapping of the filter once calibrated. 
      Omited only to allow for deployment till the calibration and proper naming of the filters.  */
    }
    else if(filter == "cone" || filter == "cones"){

    }
    else if(filter == "tag" || filter == "tags" || filter == "apriltags"){

    }
    else if (filter == "tape" || filter == "tapes"){};
  }
  // for the following -1 is an error code 
  public double getbestyaw(){
    if(getBesttarget() != null){
    return getBesttarget().getYaw();}
    else{return -1;}
  }
  public double getbestpitch(){
    if(getBesttarget() != null){return getBesttarget().getPitch();}
    else{return -1;}
    
  }
  public double getbestarea(){
    return getBesttarget().getArea();
  }
  public double getBestSkew(){
    return getBesttarget().getSkew();
  }

  public double getbestaprilid(){
    if(getBesttarget() != null && currentfilter == "tag"){
      return getBesttarget().getFiducialId();
    }
    else{
      return -1;
    }
  }
  //for the following null is an error
  public Transform3d getCamtoTarget(){
    if(getBesttarget() != null || currentfilter == "tag"){
      return getBesttarget().getBestCameraToTarget();
    }
    else{ return null;}
  }

  /*TODO Get corners appears to be deprecated. Look into replacement
  public double[] getBestCorners(){
    
    return null;

  }
  */


  //UGHHHH No transform 2d for non apriltag targets. 


  //TODO Implement robotposeestimator once possible. 


  //TODO Implement a method to convert pose3d to pose 2d, Simple as implmenting the values into a new object? 


}
