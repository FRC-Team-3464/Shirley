// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ExtenderConstants;
// import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtenderSubsystem extends SubsystemBase {

  // Positive is clockwise, and is extended. 
  private final CANSparkMax extenderMotor = new CANSparkMax(ExtenderConstants.kExtenderMotorPort, CANSparkMax.MotorType.kBrushless); // How is this working
  private final DigitalInput maxLimitSwitch = new DigitalInput(ExtenderConstants.kExtenderMaxSwitchPort);
  private final DigitalInput minLimitSwitch = new DigitalInput(ExtenderConstants.kExtenderMinSwitchPort);

  // Extender encoder
  private final RelativeEncoder extenderEncoder = extenderMotor.getEncoder();

  public ExtenderSubsystem() {
    extenderMotor.setInverted(true);
    // Get rid of this, after ensuring that the encoder values are correct. 
    extenderEncoder.setPositionConversionFactor(ExtenderConstants.kEncoderRotationToInch); // Set the conversion factor so setPosition() returns the distance in inches. 
  }
  
  /*
   * Motor methods.
   */
  
  public void extend(){
        extenderMotor.set(0.5);
  }

  public void retract(){
        extenderMotor.set(-0.8);
  };

  // Run motor continuously without any interference from limitswitch
  public void translateExtender(double speed){
    extenderMotor.set(speed);
  }

  // Stop the motor. 
  public void stopMotor(){
    extenderMotor.stopMotor();    
  }

  // Get the pivoter to the maximum position. 
  // public void goToMax(){
  //   translateExtender(0.15); // Test speed - it should stop at the max
  // }

  // // Get the pivoter to the minimum position. 
  // public void goToMin(){
  //   translateExtender(-0.15); // Test speed - it should stop at the max
  // }

  // public void manualSpeedOut(){
  //   translateExtender(0.3);
  // }

  // public void manualSpeedIn(){
  //   translateExtender(-0.3);
  // }

  // public CommandBase translateManual(double speed) {
  //   // Manual pivot command with safety from limit switches
  //   return runOnce(
  //       () -> {
  //         if(maxLimitSwitch.get() && speed > 0){ // When the max limit switch is triggered and we're trying to extend more, 
  //           extenderEncoder.setPosition(ExtenderConstants.kMaxExtensionInch); // It should be 22 inches, but will need to be changed. 
  //           extenderMotor.stopMotor();
  //         }else if(minLimitSwitch.get() && speed < 0){ // when the min limit switch is triggered and we're trying to retract more,
  //           extenderEncoder.setPosition(0); // Reset limit switch
  //           extenderMotor.stopMotor();
  //         }else{ // if neither of the switches are hit, or if one of them is hit  but we're trying to go in the opposite direciton. 
  //           extenderMotor.set(speed);
  //         }
  //       });
  // }

  // public CommandBase commandStop(){
  //   return runOnce(() -> {extenderMotor.stopMotor();});
  // }
  

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

  /*
   * Limit switch Methods 
   */

   public boolean getMaxSwitch(){
    return !maxLimitSwitch.get();
   }

   public boolean getMinSwitch(){
    return !minLimitSwitch.get();
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Extender Encoder", getExtenderPosition()); // See how many rotations are there. I may have made a mistake here
    SmartDashboard.putNumber("Extender Encoder (in)", getExtenderInchPosition());
    SmartDashboard.putBoolean("Extender MIN Limit", getMinSwitch());
    SmartDashboard.putBoolean("Extender Max Limit", getMaxSwitch());
    
  }
}
