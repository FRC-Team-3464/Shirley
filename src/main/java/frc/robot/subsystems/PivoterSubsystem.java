// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivoterConstants;

public class PivoterSubsystem extends SubsystemBase {
  // Create motor and limit switch. 
  private final CANSparkMax pivoterMotor = new CANSparkMax(PivoterConstants.kPivoterMotorPort, MotorType.kBrushless); //Right Motor for Arm Pivoter
  private final DigitalInput minLimitSwitch = new DigitalInput(PivoterConstants.kPivotMinSwitchPort);
  private final CANSparkMax secondPivoterMotor = new CANSparkMax(PivoterConstants.kPivoterSecondMotorPort, MotorType.kBrushless);

  // Get the encoder from the motor. 
  private final RelativeEncoder pivoterEncoder = pivoterMotor.getEncoder(); //Encoder for Arm Pivoter Left Motor Position (used for both)
  
  public PivoterSubsystem() {
    pivoterMotor.setInverted(false);
    // secondPivoterMotor.setInverted(false);
    secondPivoterMotor.follow(pivoterMotor, true); // This inversts the motor and tells it to follow the other. 
  }

 /*
  * Pivoter Motor methods. 
  */

  //  Run the motor to our inputted speed. 
  public void pivot(double speed){
    pivoterMotor.set(speed);
    // secondPivoterMotor.follow(pivoterMotor); // Does this work?

  }

  // Stop motor. 
  public void stopMotor(){
    pivoterMotor.stopMotor();
    secondPivoterMotor.stopMotor();
  }

  // public void pivotForward(){
  //   pivot(0.125);
  // }

  // public void pivotToMin(){
  //   pivot(-0.25); // Run pivot continously till we hit the switch, which it should do. 
  // }

  // public CommandBase pivotManual(double speed) {
  //   // Manual pivot command
  //   return runOnce(
  //       () -> {
  //         if(minLimitSwitch.get() && (Math.signum(speed) < 0)){ // if the min limit switch is triggered and we're trying to go down. 
  //           pivoterMotor.stopMotor();
  //           pivoterEncoder.setPosition(0);
  //         }else{
  //           pivoterMotor.set(speed); // Else, run the speed we want to set. 
  //         }
  //       });
  // }


  /*
   * Pivoter Encoder methods. 
   */

  public double getPivoterSpeed() {
    // Get the speed of the motor. 
    return pivoterMotor.get();
  }

  public double getPivoterRotation(){
    // Return the pivoter position in rotations. 
    return pivoterEncoder.getPosition();
  }

  public double getPivoterDegrees(){
    // Multiply the position by the conversion factor that changes it from rotation to degrees. 
    return pivoterEncoder.getPosition() * PivoterConstants.kPivoterRotationToDegree; 
  }
  
  public void resetEncoder(){
    // Set the encoder back to normal
    pivoterEncoder.setPosition(0);
  }

  public void addFeedFoward(){
    // Add some power to the pivoter to have it hold against gravity. 
    if(!getSwitch()){ // Make sure the trigger isn't activated. 
      pivot(0.05);

      // secondPivoterMotor.fo
    }
  }

  /*
   * Limit switch commands. 
   */

   public boolean getSwitch(){
    // Get the limit switch - either true or false. 
    return !minLimitSwitch.get();
   }

  @Override
  public void periodic() {
    // Print out pivoter degrees and speed
    // SmartDashboard.putBoolean("Motor Connected", pivoterMotor.con)
    SmartDashboard.putNumber("Pivoter Degrees:", getPivoterDegrees()); // get the pivoter value in degrees. 
    SmartDashboard.putNumber("Pivoter Rotations:", getPivoterRotation()); // Get the pivoter encoder rotation.
    SmartDashboard.putNumber("Pivoter Speed:", getPivoterSpeed()); // Get the speed of the pivoter. 
    SmartDashboard.putBoolean("Pivot Switch:", getSwitch()); // Get the state of the limit switch. 

  }
}
