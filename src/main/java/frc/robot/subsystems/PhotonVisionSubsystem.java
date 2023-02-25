// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */
  
  private final static PhotonCamera limelightCamera = new PhotonCamera("OV5647");
  private final static PhotonCamera aprilCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000"); // Change name. 

  public PhotonVisionSubsystem() {}
  // Return the highest target - useful (not) when trying to target the high pole on the grid. 
  public PhotonTrackedTarget getLimelightHighestTarget(){ // Turns out that this is useless 
    var limelightOutput = limelightCamera.getLatestResult();
    PhotonTrackedTarget bestTarget = limelightOutput.getBestTarget();
    if(limelightOutput.hasTargets()){ // Make sure we have targets. 
      double highestPitch = 0;
      List<PhotonTrackedTarget> targets = limelightOutput.getTargets(); // create a list of targets. 
      for(int target = 1; target < targets.size(); target++){
        PhotonTrackedTarget targetSelected = targets.get(target);
        if((targetSelected.getPitch() > highestPitch) && (Math.abs(targetSelected.getYaw()) < 60)){ // Get the highest point that fits +- 60. 
          bestTarget = targets.get(target); // This will be the current target. 
        } 
      }
      return bestTarget; // return the best target. 
    }else{
      return null;
    }      
  }

  // Return the color camera. 
  public PhotonCamera getColorCamera(){
    return aprilCamera;
  }

  // Get the limelight camera. 
  public PhotonCamera getLimelightCamera(){
    return limelightCamera;
  }
  
  public void switchIndex(PhotonCamera camera, int index){
    // When this works - we should be able to switch pipelines. 
    camera.setPipelineIndex(index);
    System.out.println(camera.getName() + "set id to " + index);
  }

  @Override
  public void periodic() {
    // Make sure that color camera is connected. 
    boolean aprilConnected = aprilCamera.isConnected();
    // Make sure that lime camera is connected. 
    boolean limeConnected = limelightCamera.isConnected();
    var limeResult = limelightCamera.getLatestResult();
    var aprilResult = aprilCamera.getLatestResult();
    boolean aprilHasTargets = aprilResult.hasTargets();
    boolean limeHasTargets = limeResult.hasTargets();

    SmartDashboard.getBoolean("Limelight", limeConnected);
    SmartDashboard.getBoolean("Limelight Targets?", limeHasTargets);
    SmartDashboard.putBoolean("AprilTag Cam", aprilConnected);
    SmartDashboard.putBoolean("AprilTag Targets?", aprilHasTargets);
  }
}
