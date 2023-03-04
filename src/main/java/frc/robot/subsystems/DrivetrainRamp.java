// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonCreator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleUnaryOperator;

public class DrivetrainRamp extends SubsystemBase implements DoubleUnaryOperator, Cloneable{
  /** Creates a new DrivetrainRamp. */

  /** The maximum allowed change in the value per second. */
  private final double 
    maxIncreasePerMillis,
    maxDecreasePerMillis;

  /** The value most recently returned. */
  private double lastValue;

  /** The time, in milliseconds, that the value most recently returned was returned at. */
  private double lastTime;

  @JsonCreator
  public DrivetrainRamp(double maxIncreasePerSecond, double maxDecreasePerSecond ) {
    this.maxIncreasePerMillis = maxIncreasePerSecond; /// 1000.0;
    this.maxDecreasePerMillis = /*maxDecreasePerSecond != null ?*/ maxDecreasePerSecond; /// 1000.0 /*: maxIncreasePerMillis*/;  
  }

 @Override
  public double applyAsDouble(double value) {
    if (value > lastValue) {  // If the value that gets passed into the ramp function is greater than the previous - if we're going forward...
      lastValue = // Return the smallest number between the speed we want it to go or (previous speed + (difference in time * the maximum increase in speed we want per miliseconds))
                  // Either way, we're getting a smaller speed.   
        Math.min(value, lastValue + (Timer.getFPGATimestamp() - lastTime) * maxIncreasePerMillis);
        //System.out.println("Accelerating");
    } else {
      // Same logic as when we're accelerating when decelerating. 
      lastValue =
          Math.max( // Get the maximum value because moving back speed is negative. 
              value, lastValue - (Timer.getFPGATimestamp() - lastTime) * maxDecreasePerMillis);
        //System.out.println("Decelerating");
    }
    lastTime = Timer.getFPGATimestamp(); // Update the speed. 
    //System.out.println(lastValue);
    return lastValue; // Return speed. 
  }

 }
