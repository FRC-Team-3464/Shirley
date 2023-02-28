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
  /** Creates a new GyroSubsystem. */
  // NavX Gyro
  private final AHRS gyro = new AHRS(Port.kMXP);

  public GyroSubsystem() {
  }


  // get our heading for trajectory tracking based on the gyro angle. 
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(gyro.getAngle()); // It's negative because we want degrees to incrase turning clockwise; the default is counterclockwise to follow the unit circle.  
  }

  // Return the gyro degrees. 
  public double getDegrees(){
    return gyro.getAngle();
  }

  // Reset our gyro. 
  public void resetGyro(){
    gyro.reset();
  }

  public void calibrateGyro(){
    gyro.calibrate();
  }

  // Return our gyro. 
  public AHRS getGyro(){
    return gyro; 
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Gyro Connection", gyro.isConnected());
    SmartDashboard.putNumber("Gyro Angle", getDegrees());

    // This method will be called once per scheduler run
  }
}
