// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class TargetRangePIDCommand extends CommandBase {
// Command to get into range of the target only. 
  private final PhotonVisionSubsystem photonSub;
  private final DrivetrainSubsystem driveSub;

  // PID constants should be tuned per robot - needs to be changed
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  private final PhotonCamera camera;
  private double forwardSpeed;
  
  public TargetRangePIDCommand(PhotonVisionSubsystem photon, DrivetrainSubsystem drive, PhotonCamera inputCamera) {
    // Use addRequirements() here to declare subsystem dependencies.
    photonSub = photon;
    driveSub = drive;
    camera = inputCamera;  
    addRequirements(driveSub);
    addRequirements(photonSub);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
    // Reset the controllers. 
    forwardController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get camera. 
      var result = camera.getLatestResult();
        if (result.hasTargets()) {
            // First calculate range
            double range =
            PhotonUtils.calculateDistanceToTargetMeters(
                    PhotonConstants.CAMERA_HEIGHT_METERS,
                    PhotonConstants.TARGET_HEIGHT_METERS,
                    PhotonConstants.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
            forwardSpeed = -forwardController.calculate(range, PhotonConstants.GOAL_RANGE_METERS);
          } 
        else{
          forwardSpeed = 0;
        }
        
        if((forwardSpeed < .2 && forwardSpeed > -.2)) {
             // If we have no targets, stay still.
             forwardSpeed = 0;
        }
        // Use our forward/turn speeds to control the drivetrain
        driveSub.arcadeDrive(forwardSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
