// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class DrivetrainConstants {
    public static final int kLeftFrontPort = 1,
        kLeftBackPort = 2,
        kRightFrontPort = 3,
        kRightBackPort = 4,
        ktrackWidthInches = 27; // We convert to meters in drivetrain sub; This might be 28 idk
  
    public static final double kRotationToInch = ((1 / 7.31) * (2 * Math.PI * 3)),
        kFeederDistance = 28; // Conversion factor from rotation
                                                                                     // to meters

  }

  public static class ExtenderConstants {
    // Ports
    public static final int kExtenderMaxSwitchPort = 0;
    public static final int kExtenderMinSwitchPort = 1;
    public static final int kExtenderMotorPort = 5;

    // Encoder constants
    public static final double kMaxExtensionInch = 22;

    // Rotation to inch conversion factor
    // public static final double kEncoderRotationToInch = (Math.PI * 2.074)*(1/20)*(16/26);
    public static final double kEncoderRotationToInch = 0.20048194349;

    // PID stuff
    public static final double tolerance = 0.75;

    // Setpoint
    public static final double kGroundExtenderValue = 9.3; // The setpoint to rotate the pivoter to the highest position.
    public static final double kLowExtenderValue = 5.75; // The setpoint to rotate the pivoter to the highest position.
    public static final double kMidExtenderConeValue = 5; // The setpoint to rotate the pivoter to the highest position.
    public static final double kMidExtenderCubeValue = 2.56;
    public static final double kHighExtenderCubeValue = 18.25;
    public static final double kHighExtenderConeValue = 21.5; // The setpoint to rotate the pivoter to the highest position. FIX

  }

  public static class GrabberConstants {
    // Ports
    public static final int kGrabberMotorPort = 7;
    public static final int kGrabberRightMotorPort = 10;
    // Encoder Conversion Factor
    public static final double kTickToDegrees = 360;

    // Grabber Speeds
    public static final double kStrongGrabSpeed = -0.3;
    public static final double kWeakGrabSpeed = -0.125;
  }

  public static class PivoterConstants {
    // Ports
    public static final int kPivoterMotorPort = 9;
    public static final int kPivoterSecondMotorPort = 6;
    public static final int kPivotMinSwitchPort = 2;

    public static final double kFeedPivoterValue = 12.6;
    public static final double kHighConePivoterValue = 18.5;
    public static final double kHighCubePivoterValue = 15.73; // The setpoint to rotate the pivoter to the highest position.
    public static final double kMidConePivoterValue = 13.9; // The setpoint to rotate the pivoter to the highest position.
    public static final double kMidCubePivoterValue = 11.15;
    public static final double kLowPivoterValue = 3; // The setpoint to rotate the pivoter to the highest position.
    public static final double kGroundPivoterValue = 0.9; // The setpoint to rotate the pivoter to the highest position.
    public static final double kGroundPivoterUpValue = 3;

    public static final double kPivoterMaxValue = 20;
    


    // Encoder Conversion Factor
    // public static final double kPivoterRotationToDegree = ((1 / 64) * (24 / 36) * 360);
    public static final double kPivoterRotationToDegree = 3.75;

  }

  public static class PhotonConstants {
    // Target photon range constants.
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24),
        TARGET_HEIGHT_METERS = Units.feetToMeters(5),
        // Angle between horizontal and the camera.
        CAMERA_PITCH_RADIANS = Units.degreesToRadians(0),
        // How far from the target we want to be
        GOAL_RANGE_METERS = Units.feetToMeters(3);
  }

  public static class UltrasonicConstants {
    public static final int pingPort = 5,
        echoPort = 6;
  }

}
