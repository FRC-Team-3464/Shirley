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
  // Create the two photonvision cameras. 
  private final static PhotonCamera limelightCamera = new PhotonCamera("OV5647"); // Limelight
  private final static PhotonCamera aprilCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000"); // Raspberry Pi on tip of grabbery. 

  public PhotonVisionSubsystem() {}
  // Return the highest target - useful (not) when trying to target the high pole on the grid; this might be useful. 
  public PhotonTrackedTarget getLimelightHighestTarget(){ 
    var limelightOutput = limelightCamera.getLatestResult(); // Get the latest results from the limelight camera. 
    PhotonTrackedTarget bestTarget = limelightOutput.getBestTarget(); // Get the best target from the last result. 
    if(limelightOutput.hasTargets()){ 
      double highestPitch = 0; 
      List<PhotonTrackedTarget> targets = limelightOutput.getTargets(); // create a list of all the targets. 
      for(int target = 1; target < targets.size(); target++){ 
        // Go through all the targets - if it's larger than the highest pitch value stored previously and it's less than 60, set it to be the best target. 
        PhotonTrackedTarget targetSelected = targets.get(target);
        if((targetSelected.getPitch() > highestPitch) && (Math.abs(targetSelected.getYaw()) < 60)){ // Get the highest point that fits +- 60. 
          bestTarget = targets.get(target); // This will be the current target. 
        } 
      }
      return bestTarget; // return the best target. 
    }else{
      return null; // With no targets - don't return anything. 
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
  
  // Switch the index; it's currently being fixed by the developers. 
  public void switchIndex(PhotonCamera camera, int index){
    // When this works - we should be able to switch pipelines. 
    camera.setPipelineIndex(index);
    // System.out.println(camera.getName() + "set id to " + index);
  }

  @Override
  public void periodic() {
    // Make sure that the camera's are connected. 
    boolean aprilConnected = aprilCamera.isConnected();
    boolean limeConnected = limelightCamera.isConnected();
    
    // See if there are targets in each of the cameras. 
    var limeResult = limelightCamera.getLatestResult();
    var aprilResult = aprilCamera.getLatestResult();
    boolean aprilHasTargets = aprilResult.hasTargets();
    boolean limeHasTargets = limeResult.hasTargets();

    // Report the results on the SmartDashboard. 
    SmartDashboard.putBoolean("Limelight:", limeConnected);
    SmartDashboard.putBoolean("Limelight Targets:", limeHasTargets);
    SmartDashboard.putNumber("Limelight Best Target X:", limeResult.getBestTarget().getYaw()); // Get the yaw - x difference of the limelight's best target. 
    SmartDashboard.putBoolean("AprilTag Targets:", aprilHasTargets);
  }
}
