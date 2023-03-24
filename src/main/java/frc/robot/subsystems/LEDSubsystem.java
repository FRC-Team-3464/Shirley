// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {
  // Create a gyroscope.
  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
  

  public LEDSubsystem() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    ledStrip = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    ledBuffer = new AddressableLEDBuffer(61);
    ledStrip.setLength(ledBuffer.getLength());

    // Set the data
    ledStrip.setData(ledBuffer);
    ledStrip.start();

  }

  public void yellow() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 243, 154, 2);
     }
     
     ledStrip.setData(ledBuffer);
     System.out.println("Yellowing");
  }

  public void red() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 200, 0, 0);
     }
     
     ledStrip.setData(ledBuffer);
     System.out.println("Redding");
  }

  public void green() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 0, 154, 30);
     }
     ledStrip.setData(ledBuffer);
     System.out.println("Greening");
  }

  public void purple() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 100, 1, 186);
     }
     
     ledStrip.setData(ledBuffer);
     System.out.println("Purpling");
  }
  

  

  /*public void rainbow() {
    // For every pixel
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the value
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += 3;
    // Check bounds
    rainbowFirstPixelHue %= 180;
  }*/



  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("Gyro Connection", gyro.isConnected());
    //SmartDashboard.putNumber("Gyro Angle", getDegrees());
  }
}
