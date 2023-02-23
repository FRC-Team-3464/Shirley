// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */
  
  // Get the camera focused on cones/cubes. 
  //  We need to update the name.
  // private final PhotonCamera objectCamera = new PhotonCamera("OV5647");
  // Camera for the reflective tape and apriltag. 
  private final PhotonCamera targetCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");  
  // private final PhotonCamera feederCamera = new PhotonCamera("feederCamera"); Need to define.

  

  public PhotonVisionSubsystem() {
    targetCamera.setPipelineIndex(1);
    System.out.println("IM ALIVE");
  }


  public void switchIndex(){
    // if(targetCamera.getPipelineIndex() == 1){
    //   targetCamera.setPipelineIndex(0);
    // }else{
      targetCamera.setPipelineIndex(1);
      System.out.println("Set to one");
    // }
  }

//   PhotonTrackedTarget target = result.getBestTarget();
//   //GENERAL CAMERA TARGET INFO
//   double yaw = target.getYaw();
//   double pitch = target.getPitch();
//   double area = target.getArea();
//   double skew = target.getSkew();
//   Transform3d pose = target.getBestCameraToTarget();
//   // List<TargetCorner> corners = target.getCorners();

//   // Get APRILTAG from target camera
//   int targetID = target.getFiducialId();
//   double poseAmbiguity = target.getPoseAmbiguity();
//   Transform3d bestCameraToTarget = target.getBestCameraToTarget();
//   Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();



//   var resultLime = cameraLime.getLatestResult();
//   boolean hasTargetsLime = resultLime.hasTargets();
//   List<PhotonTrackedTarget> targetsLime = result.getTargets();
//   System.out.println("Lime" + targetsLime);
//   CommandScheduler.getInstance().run();
// }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = targetCamera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    SmartDashboard.putBoolean("Has Targets: ", hasTargets);
    SmartDashboard.putNumber("ID", targetCamera.getPipelineIndex());
    System.out.println(targetCamera.getPipelineIndex());
  }
}
