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
  /** Creates a new GrabberSubsystem. */
  
  // Creates a CANSparkMax for the grabber motor
  private final CANSparkMax grabberMotor = new CANSparkMax(9, MotorType.kBrushless);

  // Gets encoder value of the motor
  private final RelativeEncoder grabberEncoder = grabberMotor.getEncoder();


  public GrabberSubsystem() {
    // Inverts the motor value, making it positive for opening and negative for closing
    grabberMotor.setInverted(true);
  }


  public void runMotor(double speed) { 
    // Creates a variable to run the motor
    // Sets motor to variable speed
    grabberMotor.set(speed);
  }


  public void stopMotor() {
    // Sets the speed of the motor to 0
    grabberMotor.set(0);
  }


  public double getGrabberSpeed() {
    // Returns the encoder value of the grabber motor
    return grabberMotor.get();
  }


  public double getGrabberDegrees() {
    // Gets the distance of the motor
    return (grabberEncoder.getPosition() * GrabberConstants.kTickToDegrees);
  }


  public void resetGrabberDistance() {
    // Set the position that the encoders are at to 0
    grabberEncoder.setPosition(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Puts encoder value on the SmartDashboard
    SmartDashboard.putNumber("Encoder Value Grabber Speed", getGrabberSpeed());
    SmartDashboard.putNumber("Encoder Degrees", getGrabberDegrees());

  }
}