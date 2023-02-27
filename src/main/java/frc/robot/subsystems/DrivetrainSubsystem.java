// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
  

  private PIDController forwardController = new PIDController(0, 0, 0); // WE need a turn PID and a forward PID> 
  private PIDController rotateController = new PIDController(0, 0, 0); // WE need a turn PID and a forward PID> 


  public DrivetrainSubsystem() {
    // Inverts the left motor, allowing it to go straight
    leftFront.setInverted(true); // Invert the left MOTOR only; don't invert motor and encoder at the same time. 
    // Make sure that the back motors follow the front motors. 
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    // Set the encoder conversion factor so getPosition() automatically has it converted to meters. 
    leftFrontEncoder.setPositionConversionFactor(DrivetrainConstants.kRotationToMeters); 
    rightFrontEncoder.setPositionConversionFactor(DrivetrainConstants.kRotationToMeters); 
    leftFrontEncoder.setVelocityConversionFactor(DrivetrainConstants.kRotationToMeters); 
    rightFrontEncoder.setVelocityConversionFactor(DrivetrainConstants.kRotationToMeters); 
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


  // Return the PID Controllers we need in the PID Command. 
  public PIDController getForwardController(){
    return forwardController;
  }

  public PIDController getRotationController(){
    return rotateController;
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


  public RelativeEncoder getLeftFrontRelativeEncoder(){
    return leftFrontEncoder;
  }

  public RelativeEncoder getRightFrontRelativeEncoder(){
    return rightFrontEncoder;
  }

  @Override
  public void periodic() {
    // Update our odometry to get the new heading every 20 ms. 
    // Get left and right encoder meter values - distance traveled. 
    SmartDashboard.putNumber("Left Encoder Meter Value:", getLeftPosition());
    SmartDashboard.putNumber("Right Encoder Meter Value:", getRightPosition());
  }
}