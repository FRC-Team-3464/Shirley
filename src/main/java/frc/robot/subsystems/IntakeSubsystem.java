// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  String yesObject;
  private final CANSparkMax intakeMotor = new CANSparkMax(7, MotorType.kBrushless);
  


  public IntakeSubsystem() {}

  public void runIntake(double speed) {
    intakeMotor.set(speed);
    System.out.println(intakeMotor.getOutputCurrent());
  }

  

  public double getOutputCurrent() {
    return intakeMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Output Current", intakeMotor.getOutputCurrent());
    // This method will be called once per scheduler run
  }


}
