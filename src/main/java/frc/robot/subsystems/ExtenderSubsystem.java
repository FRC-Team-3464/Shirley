// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ExtenderConstants;

public class ExtenderSubsystem extends SubsystemBase {

  // Positive is clockwise, and is extended. 
  private final CANSparkMax extenderMotor = new CANSparkMax(ExtenderConstants.kExtenderMotorPort, CANSparkMax.MotorType.kBrushless); // How is this working
  
  // Extender encoder
  private final RelativeEncoder extenderEncoder = extenderMotor.getEncoder();

  public ExtenderSubsystem() {}

  public void translateExtender(double speed){
    extenderMotor.set(speed);
  }

  public double getExtenderPosition(){
    // Return the current extender encoder position in rotation
    return extenderEncoder.getPosition(); // Return the current extender encoder position
  }

  public double getExtenderInchPosition(){
    // Return the current extender encoder position in inches. 
    return getExtenderPosition() * Constants.ExtenderConstants.kEncoderRotationToInch;
  }

  public void resetExtenderEncoder(){
    extenderEncoder.setPosition(0); // Reset encoder value to 0
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Extender Encoder", getExtenderPosition()); // See how many rotations are there. 
    SmartDashboard.putNumber("Extender Encoder (in)", getExtenderInchPosition());
    
  }
}
