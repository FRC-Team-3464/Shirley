// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class AutoDriveRotate extends CommandBase {
  /** Creates a new AutoDriveRotate. */
  private final DrivetrainSubsystem driveSub; // Leave this undefined. 
  private final GyroSubsystem gyroSub; // Leave this undefined. 
  private double directionSet; // This will be either negative or positive 1.0 based on the sign of it's argument. 
  private double target; 
  
  public AutoDriveRotate(DrivetrainSubsystem drive, GyroSubsystem gyro, double target) {
    driveSub = drive;
    gyroSub = gyro;
    this.target = target; // this refers to the target we created with private double target. 

    addRequirements(drive);
    addRequirements(gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target += gyroSub.getDegrees();
    directionSet = Math.signum(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSub.arcadeDrive(0, 0.25 * directionSet); // Make this a constant. 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.stopDrive(); // Stop drive when we're done. 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(Math.abs(target - gyroSub.getDegrees()) < 0.5); // Stop when we're close. 
  }
}
