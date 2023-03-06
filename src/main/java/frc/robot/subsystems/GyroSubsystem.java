// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class GyroSubsystem extends SubsystemBase {
  // Create a gyroscope.
  private final AHRS gyro = new AHRS(Port.kMXP);

  public GyroSubsystem() {}

  public double getDegrees(){
    // Get turn degrees 
    return gyro.getAngle();
  }

  public double getPitch(){
    // Get gyro pitch
    return gyro.getPitch();
  }

  public double getYaw(){
    // Get gyro yaw
    return gyro.getYaw();
  }

  public void resetGyro(){
    // Reset gyro. 
    gyro.reset();
  }

  public void calibrateGyro(){
    // Calibrate the gyro. 
    gyro.calibrate();
  }

  // Return our gyro. 
  public AHRS getGyro(){
    return gyro; 
  }
  
  // get our heading for trajectory tracking based on the gyro angle. 
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(gyro.getAngle()); // It's negative because we want degrees to incrase turning clockwise; the default is counterclockwise to follow the unit circle.  
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Gyro Connection", gyro.isConnected());
    SmartDashboard.putNumber("Gyro Angle", getDegrees());
  }
}
