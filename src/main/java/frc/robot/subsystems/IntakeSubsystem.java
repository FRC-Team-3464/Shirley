// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.OperatorConstants.intakeMotorPort, MotorType.kBrushless);
  


  public IntakeSubsystem() {}

  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
