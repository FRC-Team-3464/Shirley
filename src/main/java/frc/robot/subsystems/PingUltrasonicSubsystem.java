// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PingUltrasonicSubsystem extends SubsystemBase {
  /** Creates a new PingUltrasonicSubsystem. */
  // output is one, sends 
  private final Ultrasonic vexUltrasonic = new Ultrasonic(1, 0);

  public PingUltrasonicSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    vexUltrasonic.ping();
    SmartDashboard.putNumber("Ping Distance", vexUltrasonic.getRangeInches());
  }
}
