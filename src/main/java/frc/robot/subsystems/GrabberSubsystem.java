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
  private final CANSparkMax grabberMotor = new CANSparkMax(GrabberConstants.kGrabberMotorPort, MotorType.kBrushless);
  // private final CANSparkMax grabberSecondMotor = new CANSparkMax(10, MotorType.kBrushless);

  // Gets encoder of the motor
  private final RelativeEncoder grabberEncoder = grabberMotor.getEncoder();
  

  public GrabberSubsystem() {
    // Invert the grabber. 
    grabberMotor.setInverted(true);
    // grabberSecondMotor.follow(grabberMotor, true);
  }

  /*
  * Grabber methods
  */

  public void runMotor(double speed) {
    // Set motor to the passed-in speed. 
    grabberMotor.set(speed);
  }

  public void stopMotor() {
    // Stop motor
    grabberMotor.stopMotor(); 
  }

  public double getGrabberRotation() {
    // Returns the encoder value of the grabber motor in rotations
    return grabberEncoder.getPosition();
  }


  public double getGrabberDegrees() {
    // Gets the encoder value of the grabbery motor in degrees
    return (grabberEncoder.getPosition() * GrabberConstants.kTickToDegrees);
  }

  public void resetGrabberDistance() {
    // Set the position that the encoders are at to 0
    grabberEncoder.setPosition(0);
  }
  
  @Override
  public void periodic() {
    // Puts encoder values on the SmartDashboard - in degree and rotation. 
    SmartDashboard.putNumber("Grabber Rotation", getGrabberRotation()); 
    SmartDashboard.putNumber("Grabber Degrees", getGrabberDegrees());
    SmartDashboard.putNumber("Grabber Speed", grabberMotor.get()); // Give us the speed. 
    SmartDashboard.putNumber("Grabber Power", grabberMotor.getOutputCurrent());
  }
}