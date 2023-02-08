// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxAbsoluteEncoder;
// import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final CANSparkMax elevatorMotor = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless); // How is this working
  private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  // private final 

    public ElevatorSubsystem() {}

    public void elevatorUp(double speed){
      elevatorMotor.set(speed);
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder.getPosition());
  }
}