// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

public class TargetRangePIDCommand extends CommandBase {

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

  private final XboxController controller = OI.xBoxController;
  private final PhotonCamera camera;

  private double forwardSpeed;
  private double rotationSpeed;
  

  public TargetRangePIDCommand(PhotonVisionSubsystem photon, DrivetrainSubsystem drive) {
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
    //  This does not make sense - in what scenario will we be moving towards the target but also being able to free up the turn value?
        rotationSpeed = controller.getLeftX();
        // Query the latest result from PhotonVision
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            // First calculate range
            double range =
            PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS,
                    TARGET_HEIGHT_METERS,
                    CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
            // Use this range as the measurement we give to the PID controller.
            // -1.0 required to ensure positive PID controller effort _increases_ range
            forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

            // Also calculate angular power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
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
