// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {
  
  // Creates a CANSparkMax for the grabber motor
  // Clockwise closes
  private final CANSparkMax grabberMotor = new CANSparkMax(GrabberConstants.kGrabberMotorPort, MotorType.kBrushless);

  // Gets encoder value of the motor
  private final RelativeEncoder grabberEncoder = grabberMotor.getEncoder();

  public GrabberSubsystem() {
    grabberMotor.setInverted(false);
  }

  public void runMotor(double speed) { 
    // Clockwise closes
    grabberMotor.set(speed);
  }

  public void stopMotor() {
    // Sets the speed of the motor to 0
    // grabberMotor.set(0);
    grabberMotor.stopMotor(); // This might work... 
  }

  public double getGrabberRotation() {
    // Returns the encoder value of the grabber motor in rotations
    return grabberEncoder.getPosition();
  }


  public double getGrabberDegrees() {
    // Gets the encoder value of the grabbery motor in degrees
    return (grabberEncoder.getPosition() * 360); 
  }

  public void resetGrabberDistance() {
    // Set the position that the encoders are at to 0
    grabberEncoder.setPosition(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Puts encoder value on the SmartDashboard
    SmartDashboard.putNumber("Grabber Rotation", getGrabberRotation());
    SmartDashboard.putNumber("Grabber Degrees", getGrabberDegrees());

  }
}