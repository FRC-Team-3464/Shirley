// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

// import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private final CANSparkMax elevatorMotor = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless); // How is this working
  // private final 

    public ElevatorSubsystem() {}

    public void elevatorUp(double speed){
      elevatorMotor.set(speed);
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
