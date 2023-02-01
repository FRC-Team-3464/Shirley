// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

public class ColorSensorSubsystem extends SubsystemBase {
  /** Creates a new ColorSensorSubsystem. */
  private final I2C.Port i2cPort = I2C.Port.kOnboard; // I2C port on roborio
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private final ColorMatch m_colorMatch = new ColorMatch();
  
  // private final Color kBlueTarget = new Color(0.143,0.427,0.429);
  // private final Color kRedTarget = new Color(kBlueTarget),0.427,0.429);
  // private final Color kYellowTarget = new Color(0.143,0.427,0.429);
  // m_colorSensor

  public ColorSensorSubsystem() {}

  @Override
  public void periodic() {
    // SmartDashboard.putNumberArray("Color RGB Values", m_colorSensor.getColor());
    SmartDashboard.putNumber("Red Value", m_colorSensor.getRed());
    SmartDashboard.putNumber("Green Value", m_colorSensor.getGreen());
    SmartDashboard.putNumber("Blue Value", m_colorSensor.getBlue());

    Color detectedColor = m_colorSensor.getColor();
    Color8Bit rgbColor = new Color8Bit(detectedColor);

    SmartDashboard.putString("Color Value", rgbColor.toString());

    // SmartDashboard.putNumberArray("Blue Value", [m_colorSensor.getRed()] m_colorSensor.getBlue());

    // SmartDashboard.putString("Color", String(m_colorSensor.getColor())); 
  }
}