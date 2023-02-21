// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */
  
  // Defining the drive train motors
  
  // Motor Ports will be changed later
  private final CANSparkMax
    leftFront = new CANSparkMax(DrivetrainConstants.kLeftFrontPort, MotorType.kBrushless),
    leftBack = new CANSparkMax(DrivetrainConstants.kLeftBackPort, MotorType.kBrushless),
    rightFront = new CANSparkMax(DrivetrainConstants.kRightFrontPort, MotorType.kBrushless),
    rightBack = new CANSparkMax(DrivetrainConstants.kRightBackPort, MotorType.kBrushless);

  // Gets encoder values from the two front motors
  private final RelativeEncoder leftFrontEncoder = leftFront.getEncoder();
  private final RelativeEncoder rightFrontEncoder = rightFront.getEncoder();

  // Differential drive, allows arcade drive and tank drive
  private DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);
  private final AHRS gyro = new AHRS(Port.kMXP);

  // Create drive kinemeatics
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DrivetrainConstants.ktrackWidthInches)); // Might be 27
  // Create drive odometry
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());

  // Store our robot position with Pose - contains x, y and heading.
  private Pose2d pose; 

  public DrivetrainSubsystem() {
    // Inverts the left motor, allowing it to go straight
    leftFront.setInverted(true);
    leftFrontEncoder.setPositionConversionFactor(DrivetrainConstants.kRotationToMeters); // Set it our rotation to meters conversion factor so it applies to .getPosition()
    rightFrontEncoder.setPositionConversionFactor(DrivetrainConstants.kRotationToMeters); // I believe that our gear ratio is 7.31:1
    
  }

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-gyro.getAngle()); // It's negative because we want degrees to incrase turning clockwise; the default is counterclockwise to follow the unit circle.  
  }
  
  public void driveTank(double left, double right) {
    // Gets rid of the joystick drift
    if (Math.abs(left) < 0.07) {
      left = 0;
    }
    if (Math.abs(right) < 0.07) {
      right = 0;
    }

    // Uses the existing tankDrive() from the DifferentialDrive along with modified values
    drive.tankDrive(left, right);

    // Makes the back motors move the same speed as the front
    leftBack.set(leftFront.get());
    rightBack.set(rightFront.get());
  }

  public void arcadeDrive(double speed, double rotation) {
    // Uses the built in arcadeDrive() from DifferentialDrive
    drive.arcadeDrive(speed, rotation);
    // Have back motors follow the front motors
    leftBack.set(leftFront.get());
    rightBack.set(rightFront.get());
  }
  
  public void stopDrive() {
    // Stops the arcadeDrive
    arcadeDrive(0, 0);
  }


  public double getLeftSpeed() {
    // Returns the left encoder value
    return leftFront.get();
  }

  public double getRightSpeed() {
    // Returns the right encoder value
    return rightFront.get();
  }

  public double getForwardSpeed() {
    // Returns the average value of both encoders
    return ((getLeftSpeed() + getRightSpeed()) /2 );
  }

  public void resetEncoders() {
    // Sets the position the motors are at to 0
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
  }

  public double getForwardDistance() {
    // Gets the average position of the two encoders
    return ((leftFrontEncoder.getPosition() + rightFrontEncoder.getPosition() /2 ));
  }

  public void enableMotors(boolean on) {
    // Sets the motors to either kBrake (no movement) or kCoast (can push to move)
    // if Mode is on, then kBrake will be on, elsem kCoast will be on
    IdleMode mode;
    if(on) {
      mode = IdleMode.kBrake;
    } else {
      mode = IdleMode.kCoast;
    }
    // Sets the motors to the mode activated
    leftFront.setIdleMode(mode);
    leftBack.setIdleMode(mode);
    rightFront.setIdleMode(mode);
    rightBack.setIdleMode(mode);
  }


  @Override
  public void periodic() {
    // Update our odometry to get the new heading every 20 ms. 
    pose = odometry.update(getHeading(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());

    // Get left and right encoder meter values - distance traveled. 
    SmartDashboard.putNumber("Left Encoder Meter Value:", leftFrontEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder Meter Value:", rightFrontEncoder.getPosition());

  }
}