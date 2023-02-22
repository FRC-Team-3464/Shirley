// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */
  
  // Get the camera focused on cones/cubes. 
  //  We need to update the name.
  private final PhotonCamera objectCamera = new PhotonCamera("objectCamera");
  
  // Camera for the reflective tape and apriltag. 
  private final PhotonCamera targetCamera = new PhotonCamera("limelightCamera");
  
  private final PhotonCamera feederCamera = new PhotonCamera("feederCamera");
  

  public PhotonVisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
