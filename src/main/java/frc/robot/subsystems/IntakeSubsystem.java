// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  boolean Object;
  private final CANSparkMax intakeMotor = new CANSparkMax(7, MotorType.kBrushless);

  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();


  public IntakeSubsystem() {
    Object = false;
    intakeEncoder.setPosition(0);
  }

  public void runIntake(double speed) {
    intakeMotor.set(speed);
    if(speed < 0) {
      Object = false;
    }
    if(intakeMotor.getOutputCurrent() > 40) {
      Object = true;
    }
    System.out.println(intakeMotor.getOutputCurrent());
    //System.out.println(Object);
  }


  public boolean yesObject() {
    return Object;
  }

  public double getSpeed() {
    return intakeMotor.get();
  }

  public double getEncoder() {
    return intakeEncoder.getPosition();
  }

  public double getOutputCurrent() {
    return intakeMotor.getOutputCurrent();
  }

  public void stopMotor() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Output Current", intakeMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Piece??????", Object);
    SmartDashboard.putNumber("Intake Encoder", intakeEncoder.getPosition());
    // This method will be called once per scheduler run
  }


}
