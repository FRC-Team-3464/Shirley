// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */
  
  // Defining the drive train motors
  
  // Motor Ports will be changed later
  
  private final CANSparkMax
    leftFront = new CANSparkMax(3, null),
    leftBack = new CANSparkMax(4, null),
    rightFront = new CANSparkMax(5, null),
    rightBack = new CANSparkMax(6, null);
  
  // Gets encoder values from the two front motors
  private final RelativeEncoder leftFrontEncoder = leftFront.getEncoder();
  private final RelativeEncoder rightFrontEncoder = rightFront.getEncoder();

  
  public DriveTrainSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
