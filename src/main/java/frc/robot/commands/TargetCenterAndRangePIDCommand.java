// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class TargetCenterAndRangePIDCommand extends CommandBase {
// Auto center and get in range of a target simultaneously. 
  private final PhotonVisionSubsystem photonSub;
  private final DrivetrainSubsystem driveSub;

  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  // PID constants should be tuned per robot - needs to be changed
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  private final PhotonCamera camera;
  private double forwardSpeed;
  private double rotationSpeed;
  
  public TargetCenterAndRangePIDCommand(PhotonVisionSubsystem photon, DrivetrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    photonSub = photon;
    driveSub = drive;
    camera = photonSub.getColorCamera();  
    addRequirements(driveSub);
    addRequirements(photonSub);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
    // Reset the controllers. 
    forwardController.setSetpoint(0);
    turnController.setSetpoint(0); 
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
                    CAMERA_HEIGHT_METERS,
                    TARGET_HEIGHT_METERS,
                    CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
            forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
            rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
          } 
        else{
          forwardSpeed = 0;
          rotationSpeed = 0;
        }
        
        if((forwardSpeed < .2 && forwardSpeed > -.2)) {
             // If we have no targets, stay still.
             forwardSpeed = 0;
        }
        if((rotationSpeed < .2 && rotationSpeed > -.2)) {
          // If we have no targets, stay still.
          forwardSpeed = 0;
        }

        // Use our forward/turn speeds to control the drivetrain
        driveSub.arcadeDrive(forwardSpeed, rotationSpeed);
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
