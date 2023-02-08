// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicSubsystem extends SubsystemBase {
  /** Creates a new UltrasonicSubsystem. */
  public UltrasonicSubsystem() {}

  // AnalogInput input = new AnalogInput(0)
  AnalogPotentiometer distanceAnalog = new AnalogPotentiometer(0,100,0);

  @Override
  public void periodic() {
    SmartDashboard.putNumber("distance", distanceAnalog.get());
    // This method will be called once per scheduler run
  }
}
