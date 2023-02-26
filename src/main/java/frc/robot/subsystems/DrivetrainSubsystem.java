// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */
  // Defining the drive train motors
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
  
  // NavX Gyro
  private final AHRS gyro = new AHRS(Port.kMXP);

  // Follow along Green Hope Falcon's Trajectory Tracking Tutorial: 
  // Create drive kinemeatics
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DrivetrainConstants.ktrackWidthInches)); // Might be 27
  // Create drive odometry
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());
  // Get a feedforward 
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.268, 1.89, 0.243); // We need to run frc-characterization in order to do this. 
  private final PIDController leftPIDController = new PIDController(9.95, 0, 0); // Also need to get values from frc-characterization. 
  private final PIDController rightPIDController = new PIDController(9.95, 0, 0); // Also need to get values from frc-characterization. 
 
  // Store our robot position with Pose - contains x, y and heading.
  private Pose2d pose; 

  public DrivetrainSubsystem() {
    // Inverts the left motor, allowing it to go straight
    leftFront.setInverted(true); // Invert the left MOTOR only; don't invert motor and encoder at the same time. 
    // Make sure that the back motors follow the front motors. 
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    // Set the encoder conversion factor so getPosition() automatically has it converted to meters. 
    leftFrontEncoder.setPositionConversionFactor(DrivetrainConstants.kRotationToMeters); // Set it our rotation to meters conversion factor so it applies to .getPosition()
    rightFrontEncoder.setPositionConversionFactor(DrivetrainConstants.kRotationToMeters); // I believe that our gear ratio is 7.31:1
    leftFrontEncoder.setVelocityConversionFactor(DrivetrainConstants.kRotationToMeters); // Set it our rotation to meters conversion factor so it applies to .getPosition()
    rightFrontEncoder.setVelocityConversionFactor(DrivetrainConstants.kRotationToMeters); // I believe that our gear ratio is 7.31:1

  }

  /*
   * Drivetrain Methods
   */

  public void driveTank(double left, double right) {
    // Gets rid of the joystick drift
    if (Math.abs(left) < 0.07) { // Update to be the volts needed to overcome friction. 
      left = 0; 
    }
    if (Math.abs(right) < 0.07) {
      right = 0;
    }
    // Uses the existing tankDrive() from the DifferentialDrive along with modified values
    drive.tankDrive(left, right);
  }

  public void arcadeDrive(double speed, double rotation) {
    // Uses the built in arcadeDrive() from DifferentialDrive
    drive.arcadeDrive(speed, rotation);
  }
  
  public void tankDrive(double leftSpeed, double rightSpeed){
    // Move the driveSub based on the passed in left and right speeds. 
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void stopDrive() {
    // Stops the arcadeDrive
    drive.stopMotor(); 
  }

  /*
   * Encoder Methods 
   */

  public double getLeftSpeed() {
    // Should be velocity in m/s
    return leftFrontEncoder.getVelocity();
  }

  public double getRightSpeed() {
    // Should be velocity in m/s
    return rightFrontEncoder.getVelocity();
  }

  // Create get left and right positions - should be in meters. 
  public double getLeftPosition(){
    // Should be in meters. 
    return leftFrontEncoder.getPosition();
  }

  public double getRightPosition(){
    // Should be in meters. 
    return rightFrontEncoder.getPosition();
  }

  public void resetEncoders() {
    // Sets the position the motors are at to 0
    leftFrontEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
  }

  public void enableMotors(boolean on) {
    // Sets the motors to either kBrake (no movement) or kCoast (can push to move)
    // if Mode is on, then kBrake will be on, else if on is false, kCoast will be on
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

    /*
   * Localization/Trajectory Tracking Methods. 
   */

  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-gyro.getAngle()); // It's negative because we want degrees to incrase turning clockwise; the default is counterclockwise to follow the unit circle.  
  }

  // Return the pose, which is suppoed odometry that's fed in our newloy updated heading and left/right positions. 
  public Pose2d getPose(){
    return pose; // We should be able to get it from periodic()...
  }

  // Return the drivewheel speeds - which are useful for creating the ramsete controller in robotContainer. 
  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      leftFrontEncoder.getVelocity() * DrivetrainConstants.kRotationToMeters,
      rightFrontEncoder.getVelocity() * DrivetrainConstants.kRotationToMeters);
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  // Return the feedforward as well as the left and right PID Controllers. 
  public SimpleMotorFeedforward getFeedforward(){
    return feedforward;
  }

  public PIDController getLeftPIDController(){
    // Returns the PID Controller for the left side. 
    return leftPIDController;
  }
  
  public PIDController getRightPIDController(){
    // Returns the PID Controller for the right side. 
    return rightPIDController;
  }
  
  public void setDriveMotorVolts(double leftVolts, double rightVolts){
    leftFront.set(leftVolts / 12); // Covert from volts to speeds we input to the sparkMax. 
    rightFront.set(rightVolts / 12);
  }

  @Override
  public void periodic() {
    // Update our odometry to get the new heading every 20 ms. 
    // Get left and right encoder meter values - distance traveled. 
    SmartDashboard.putNumber("Left Encoder Meter Value:", getLeftPosition());
    SmartDashboard.putNumber("Right Encoder Meter Value:", getRightPosition());
  }
}