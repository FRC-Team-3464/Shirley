// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ExtenderConstants;

public class ExtenderSubsystem extends SubsystemBase {

  // Positive is clockwise, and is extended. 
  private final CANSparkMax extenderMotor = new CANSparkMax(ExtenderConstants.kExtenderMotorPort, CANSparkMax.MotorType.kBrushless); // How is this working
  private final DigitalInput maxLimitSwitch = new DigitalInput(ExtenderConstants.maxLimitSwitchPort);
  private final DigitalInput minLimitSwitch = new DigitalInput(ExtenderConstants.minLimitSwitchPort);

  // Extender encoder
  private final RelativeEncoder extenderEncoder = extenderMotor.getEncoder();

  public ExtenderSubsystem() {
    extenderEncoder.setPositionConversionFactor(ExtenderConstants.kEncoderRotationToInch); // Set the conversion factor so setPosition() returns the distance in inches. 
  }

  /*
   * Motor methods.
   */

  public void translateExtender(double speed){
    // Logic to control when we hit the limit switch or not. 
    if(maxLimitSwitch.get()){ // When the max limit switch is triggered. 
      extenderEncoder.setPosition(ExtenderConstants.maxExtensionInch); // It should be 22 inches, but will need to be changed. 
      extenderMotor.stopMotor();
    }else if(minLimitSwitch.get()){ // when the min limit switch is triggered. 
      extenderEncoder.setPosition(0);
      extenderMotor.stopMotor();
    }else{ // if neither of the switches are hit. 
      extenderMotor.set(speed);
    }
  }


  public void stopMotor(){
    extenderMotor.stopMotor(); // Stop the motor.     
  }

  /*
   * Encoder methods.
   */

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
