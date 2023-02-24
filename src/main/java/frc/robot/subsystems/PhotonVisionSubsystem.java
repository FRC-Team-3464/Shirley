// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */
  
  private final PhotonCamera limelightCamera = new PhotonCamera("OV5647");
  private final PhotonCamera aprilCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000"); 

  public PhotonVisionSubsystem() {}
  
  // Return the color camera. 
  public PhotonCamera getColorCamera(){
    return aprilCamera;
  }

  public PhotonCamera getLimelightCamera(){
    return limelightCamera;
  }

  public void switchIndex(PhotonCamera camera, int num){
    // When this works - we should be able to switch pipelines. 
    camera.setPipelineIndex(num);
    System.out.println(camera.getName() + "set ");
  }

  @Override
  public void periodic() {
    // Make sure that color camera is connected. 
    boolean aprilConnected = aprilCamera.isConnected();
    var aprilResult = aprilCamera.getLatestResult();
    boolean aprilHasTargets = aprilResult.hasTargets();
    SmartDashboard.putBoolean("AprilTag Cam", aprilConnected);
    SmartDashboard.putBoolean("AprilTag Has Targets: ", aprilHasTargets);
  }
}
