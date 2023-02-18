// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicSubsystem extends SubsystemBase {
  /** Creates a new UltrasonicSubsystem. */
  DigitalOutput ping = new DigitalOutput(1);
  DigitalInput echo = new DigitalInput(2);
  private final Ultrasonic vexUltrasonic = new Ultrasonic(ping, echo);
  
  public UltrasonicSubsystem() {
    // Turn on Ultrasonic sensor
    Ultrasonic.setAutomaticMode(true);
  }

  @Override
  public void periodic() {
    double distanceInches = vexUltrasonic.getRangeInches();
    SmartDashboard.putNumber("Ping Distance", distanceInches);
  }
}
