// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
