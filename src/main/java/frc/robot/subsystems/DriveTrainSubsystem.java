// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */
  
  // Defining the drive train motors
  
  // Motor Ports will be changed later
  private final CANSparkMax
    leftFront = new CANSparkMax(3, null),
    leftBack = new CANSparkMax(4, null),
    rightFront = new CANSparkMax(5, null),
    rightBack = new CANSparkMax(6, null);
  
  // Gets encoder values from the two front motors
  private final RelativeEncoder leftFrontEncoder = leftFront.getEncoder();
  private final RelativeEncoder rightFrontEncoder = rightFront.getEncoder();

  // Differential drive, allows arcade drive and tank drive
  public DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);



  
  public DriveTrainSubsystem() {
    // Inverts the left motor, allowing it to go straight
    leftFront.setInverted(true);
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
    // This method will be called once per scheduler run
    // Puts each values for the numbers on Smart Dashboard
    SmartDashboard.putNumber("Encoder Forward Distance", getForwardDistance());
    SmartDashboard.putNumber("Encoder Left Speed", getLeftSpeed());
    SmartDashboard.putNumber("Encoder Right Speed", getRightSpeed());
    SmartDashboard.putNumber("Forward Distance (feet)", getForwardDistance());
  }
}
