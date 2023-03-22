// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class TargetCenterPIDCommand extends CommandBase {
  private final PhotonVisionSubsystem photonSub;
  private final DrivetrainSubsystem driveSub;

// PID constants should be tuned per robot - GET THIS UPDATED
//   final double LINEAR_P = 0.1;
//   final double LINEAR_D = 0.0;
//   PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

// Get our angular P and D values. 
  final double turnP = 0.025; // Max yaw error is 30, which Yifan sets to 80% so 0.8/30 = 0.0266..
  final double turnD = 0.0;
  PIDController turnController = new PIDController(turnP, 0, turnD);

  private final XboxController controller = OI.xBoxController;
  // private final PhotonCamera camera;
  private double forwardSpeed;
  private double rotationSpeed;

  public TargetCenterPIDCommand(PhotonVisionSubsystem photon, DrivetrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    photonSub = photon;
    driveSub = drive;

    // camera = inputCamera;  
    addRequirements(driveSub);
    addRequirements(photonSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset controllers. 
    // forwardController.setSetpoint(0);
    turnController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    forwardSpeed = controller.getLeftY(); // Move the forward speed by the left value of the controller. 
    // Vision-alignment mode
    var result = photonSub.getLimelightCamera().getLatestResult(); // Get the most recent result from the camera. 

    if (result.hasTargets()) {
        // Calculate angular turn power
        rotationSpeed = -turnController.calculate(result.getTargets().get(0).getYaw(), 0); // Calculate it from current position to 0. 
      } 
    else{
      rotationSpeed = 0; // Don't do anything if there's no targets. 
    }
    
    if(rotationSpeed < .2 && rotationSpeed > -.2) {
      // If the rotation speed is too small - set it to zero.
        rotationSpeed = 0;
    }
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
