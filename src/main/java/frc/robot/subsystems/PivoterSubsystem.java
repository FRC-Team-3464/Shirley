// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.PivoterConstants;
import frc.robot.Constants.PivoterConstants;

public class PivoterSubsystem extends SubsystemBase {
  /** Creates a new ArmPivoterSubsystem. */
  
  private final CANSparkMax
    leftPivoter = new CANSparkMax(PivoterConstants.kPivoterLeftMotorPort, MotorType.kBrushless), //Left Motor for Arm Pivoter
    rightPivoter = new CANSparkMax(PivoterConstants.kPivoterRightMotorPort, MotorType.kBrushless); //Right Motor for Arm Pivoter

  private final RelativeEncoder leftPivotEncoder = leftPivoter.getEncoder(); //Encoder for Arm Pivoter Left Motor Position (used for both)
  
  public PivoterSubsystem() {
    // Which way should the pivoter rotate? 
    leftPivoter.setInverted(true);
  }


  public void pivot(double speed) { 
    rightPivoter.set(speed);
    leftPivoter.follow(rightPivoter);
  }

  public double getPivoterSpeed() {
    return rightPivoter.get();
  }

  public double getPivoterTicks(){
    return leftPivotEncoder.getPosition();
  }

  public double getPivoterDegrees(){
    // Multiply the position  - in ticks - by the conversion factor that changes it from ticks to degrees. 
    return leftPivotEncoder.getPosition() * PivoterConstants.kPivoterTickToDegree; 
  }

  public void resetEncoders(){
    leftPivotEncoder.setPosition(0);
  }


  @Override
  public void periodic() {
    // Print out pivoter degrees and speed
    SmartDashboard.putNumber("Pivoter Degrees", getPivoterDegrees());
    SmartDashboard.putNumber("Pivoter Speed", getPivoterSpeed());

  }
}
