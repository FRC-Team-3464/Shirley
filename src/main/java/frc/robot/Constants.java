// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class DrivetrainConstants{
    public static final int 
      kLeftFrontPort = 4,
      kLeftBackPort = 5,
      kRightFrontPort = 6,
      kRightBackPort = 7,

     ktrackWidthInches = 27; // We convert to meters in drivetrain sub; This might be 28 idk
    public static final double kRotationToMeters = ((1/7.31) * (2 * Math.PI * Units.inchesToMeters(3))); // Conversion factor from rotation to meters
  
  
  }
  public static class PivoterConstants{
    public static final double kPivoterRotationToDegree = ((1/64) *  (24/36)  *360);
    public static final int kPivoterLeftMotorPort = 1;
    public static final int kPivoterRightMotorPort = 3;

  }

  public static class ExtenderConstants{
    public static final int kExtenderMotorPort = 2;
    // 3 1/2 rotations  = max extention. 
    public static final double kEncoderRotationToInch = ((Math.PI * 2.074) * (1/20) * (16/26)); // Math.PI * 2.074 is the Pitch Diameter, (1/20) * (16/26) are gear ratios. 
    public static final double tolerance = 0.75; // How much we'll tolerate from the set commands. 
  }

  public static class GrabberConstants {
    public static final int kGrabberMotorPort = 8;
    }
}
