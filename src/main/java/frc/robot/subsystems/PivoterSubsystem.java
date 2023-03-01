// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivoterConstants;

public class PivoterSubsystem extends SubsystemBase {
  /** Creates a new ArmPivoterSubsystem. */
  // Clockwise spins down that moves the pivoter down
  
  private final CANSparkMax pivoterMotor = new CANSparkMax(PivoterConstants.kPivoterMotorPort, MotorType.kBrushless); //Right Motor for Arm Pivoter
  private final DigitalInput minLimitSwitch = new DigitalInput(PivoterConstants.kPivotMinSwitchPort);

  private final RelativeEncoder pivoterEncoder = pivoterMotor.getEncoder(); //Encoder for Arm Pivoter Left Motor Position (used for both)
  
  public PivoterSubsystem() {
    pivoterMotor.setInverted(false);
  }

 /*
  * Pivoter Motor methods. 
  */

  //   Return a command 
  public CommandBase pivotManual(double speed) {
    // Manual pivot command
    return runOnce(
        () -> {
          if(minLimitSwitch.get() && (Math.signum(speed) < 0)){ // if the min limit switch is triggered and we're trying to go down. 
            pivoterMotor.stopMotor();
            pivoterEncoder.setPosition(0);
          }else{
            pivoterMotor.set(speed); // Else, run the speed we want to set. 
          }
        });
  }

//  Run unrestrained - for commands. 
  public void pivot(double speed){
    pivoterMotor.set(speed);
  }


  // public void pivotToMin(){
  //   pivot(-0.25); // Run pivot continously till we hit the switch, which it should do. 
  // }

  public void stopMotor(){
    pivoterMotor.stopMotor(); // Stop motor. 
  }

  /*
   * Pivoter Encoder methods. 
   */

  public double getPivoterSpeed() {
    return pivoterMotor.get();
  }

  public double getPivoterRotation(){
    // Return the pivoter position in rotations. 
    return pivoterEncoder.getPosition();
  }

  public double getPivoterDegrees(){
    // Multiply the position  - in ticks - by the conversion factor that changes it from ticks to degrees. 
    return pivoterEncoder.getPosition() * PivoterConstants.kPivoterRotationToDegree; 
  }
  
  public void resetEncoder(){
    // Set the encoder back to normal
    pivoterEncoder.setPosition(0);
  }

  /*
   * Limit switch commands. 
   */

   public boolean getSwitch(){
    return minLimitSwitch.get();
   }


  @Override
  public void periodic() {
    // Print out pivoter degrees and speed
    SmartDashboard.putNumber("Pivoter Degrees", getPivoterDegrees());
    SmartDashboard.putNumber("Pivoter Rotations", getPivoterRotation());
    SmartDashboard.putNumber("Pivoter Speed", getPivoterSpeed());
    SmartDashboard.putBoolean("Pivot Switch", getSwitch());

  }
}
