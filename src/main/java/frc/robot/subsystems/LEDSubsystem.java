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
  String ledState;

  

  public LEDSubsystem() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    ledStrip = new AddressableLED(9);
    ledState = "starting";

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    ledBuffer = new AddressableLEDBuffer(61);
    ledStrip.setLength(ledBuffer.getLength());

    // Set the data
    ledStrip.setData(ledBuffer);
    ledStrip.start();

  }

  public String getLEDState(){
    return ledState;
  }


  public void yellow() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 243, 154, 2);
     }
     
     ledStrip.setData(ledBuffer);
     System.out.println("Yellowing");
     ledState = "cone";
  }

  public void purple() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 100, 1, 186);
     }
     
     ledStrip.setData(ledBuffer);
     ledState = "cube";
     System.out.println("Purpling");
  }

  public void white() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 74, 65, 42);
     }
     
     ledStrip.setData(ledBuffer);
     System.out.println("Browinign");
  }

  public void blue() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 5, 20, 250);
     }
     
     ledStrip.setData(ledBuffer);
     System.out.println("Blueing");
  }

  public void red() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 200, 20, 15);
     }
     
     ledStrip.setData(ledBuffer);
     System.out.println("Redding");
  }

  public void green() {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        ledBuffer.setRGB(i, 0, 194, 30);
     }
     ledStrip.setData(ledBuffer);
     System.out.println("Greening");
  }




  public void rainbow() {
    for (var i = 0; i < 8; i++) { // fIRST 8 leds. 
      ledBuffer.setRGB(i, 100, 1, 0);
    }
    for (var i = 8; i < 16; i++) { // fIRST 8 leds. 
      ledBuffer.setRGB(i, 130, 50, 0);
    } 
    for (var i = 16; i < 24; i++) { // fIRST 8 leds. 
      ledBuffer.setRGB(i, 100,100, 0);
    }
    for (var i = 24; i < 32; i++) { // fIRST 8 leds. 
      ledBuffer.setRGB(i, 0,100, 0);
    }
    for (var i = 32; i < 40; i++) { // fIRST 8 leds. 
      ledBuffer.setRGB(i, 0,0, 100);
    }
    for (var i = 40; i < ledBuffer.getLength(); i++) { // fIRST 8 leds. 
      ledBuffer.setRGB(i, 100, 1, 186);
    }

    ledStrip.setData(ledBuffer);
}





  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("Gyro Connection", gyro.isConnected());
    //SmartDashboard.putNumber("Gyro Angle", getDegrees());
    SmartDashboard.putString("LedState", ledState);
  }
}
