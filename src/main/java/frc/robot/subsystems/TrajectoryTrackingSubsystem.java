// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class TrajectoryTrackingSubsystem extends SubsystemBase {
  /** Creates a new TrajectoryTrackingSubsystem. */
  
  // Create drive kinemeatics
  private final DrivetrainSubsystem driveSub;
  private final GyroSubsystem gyroSub;

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DrivetrainConstants.ktrackWidthInches)); // Might be 27
  // Create drive odometry
  private DifferentialDriveOdometry odometry;
  // Get a feedforward 
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.268, 1.89, 0.243); // We need to run frc-characterization in order to do this. 
  private final PIDController leftPIDController = new PIDController(9.95, 0, 0); // Also need to get values from frc-characterization. 
  private final PIDController rightPIDController = new PIDController(9.95, 0, 0); // Also need to get values from frc-characterization. 
 
  // Store our robot position with Pose - contains x, y and heading.
  private Pose2d pose; 


  public TrajectoryTrackingSubsystem(DrivetrainSubsystem drive, GyroSubsystem gyro) {
    driveSub = drive;
    gyroSub = gyro;
  }

  // Return the drivewheel speeds - which are useful for creating the ramsete controller in robotContainer. 
  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      driveSub.getLeftSpeed() * DrivetrainConstants.kRotationToMeters,
      driveSub.getRightSpeed() * DrivetrainConstants.kRotationToMeters);
  }

  public void setDriveMotorVolts(double leftVolts, double rightVolts){
    driveSub.tankDrive(leftVolts / 12, rightVolts / 12);
  }

  public Pose2d getPose(){
    return pose; // We should be able to get it from periodic()...
  }


  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  // Return the feedforward as well as the left and right PID Controllers. 
  public SimpleMotorFeedforward getFeedforward(){
    return feedforward;
  }

  public PIDController getLeftPIDController(){
    // Returns the PID Controller for the left side. 
    return leftPIDController;
  }
  
  public PIDController getRightPIDController(){
    // Returns the PID Controller for the right side. 
    return rightPIDController;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry =  new DifferentialDriveOdometry(gyroSub.getHeading(), driveSub.getLeftPosition(), driveSub.getRightPosition());
  }
}
