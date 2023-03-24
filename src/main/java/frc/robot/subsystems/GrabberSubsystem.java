// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.GrabberConstants;

public class GrabberSubsystem extends SubsystemBase {
  // Creates a CANSparkMax for the grabber motor - Need to verify direction. 
  private final CANSparkMax lGrabberMotor = new CANSparkMax(GrabberConstants.kGrabberMotorPort, MotorType.kBrushless);
  private final CANSparkMax rGrabberMotor = new CANSparkMax(GrabberConstants.kGrabberRightMotorPort, MotorType.kBrushless);

  // Gets encoder of the motor
  private final RelativeEncoder lGrabberEncoder = lGrabberMotor.getEncoder();
  private final RelativeEncoder rGrabberEncoder = rGrabberMotor.getEncoder();
  


  public GrabberSubsystem() {
    // Invert the grabber. 
    lGrabberMotor.setInverted(true);
    rGrabberMotor.follow(lGrabberMotor, true);
  }

  /*
  * Grabber methods
  */

  public void runMotor(double speed) {
    // Set motor to the passed-in speed. 
    lGrabberMotor.set(speed);
  }

  public void stopMotor() {
    // Stop motor
    lGrabberMotor.stopMotor(); 
    System.out.println("motor stopped");
  }

  public double getGrabberRotation() {
    // Returns the encoder value of the grabber motor in rotations
    return lGrabberEncoder.getPosition();
  }


  public double getGrabberDegrees() {
    // Gets the encoder value of the grabbery motor in degrees
    return (lGrabberEncoder.getPosition() * GrabberConstants.kTickToDegrees);
  }

  public void resetGrabberDistance() {
    // Set the position that the encoders are at to 0
    lGrabberEncoder.setPosition(0);
    rGrabberEncoder.setPosition(0);
  }
  
  @Override
  public void periodic() {
    // Puts encoder values on the SmartDashboard - in degree and rotation. 
    SmartDashboard.putNumber("Left Grabber Rotation", getGrabberRotation()); 
    SmartDashboard.putNumber("Left Grabber Degrees", getGrabberDegrees());
    SmartDashboard.putNumber("Right Grabber Rotation", rGrabberEncoder.getPosition());
    SmartDashboard.putNumber("Right Grabber Degree", rGrabberEncoder.getPosition() * GrabberConstants.kTickToDegrees);
    
    SmartDashboard.putNumber("Grabber Speed", lGrabberMotor.get()); // Give us the speed. 
    SmartDashboard.putNumber("Grabber Power", lGrabberMotor.getOutputCurrent());

    SmartDashboard.putNumber("Left Motor Temperature", lGrabberMotor.getMotorTemperature());
    SmartDashboard.putNumber("Right Motor Temperature", rGrabberMotor.getMotorTemperature());
  }
}