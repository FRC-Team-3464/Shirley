// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;

public class ColorSensorSubsystem extends SubsystemBase {
  /** Creates a new ColorSensorSubsystem. */
  private final I2C.Port i2cPort = I2C.Port.kOnboard; // I2C port on roborio
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  

  public ColorSensorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
