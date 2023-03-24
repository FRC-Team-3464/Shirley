package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class UltrasonicSubsystem extends SubsystemBase {
  /** Creates a new UltrasonicSubsystem. */

 DigitalOutput ping = new DigitalOutput(8);
  DigitalInput echo = new DigitalInput(9);
  private final Ultrasonic vexUltrasonic = new Ultrasonic(ping, echo);

  public UltrasonicSubsystem() {
    // Turn on Ultrasonic sensor
    Ultrasonic.setAutomaticMode(true);
     }

  public double getUltraDistance(){
    return vexUltrasonic.getRangeInches();
  }

  public boolean getAtDistance(){
    return  getUltraDistance() < DrivetrainConstants.kFeederDistance && getUltraDistance() > DrivetrainConstants.kFeederDistance - 1;
  }

  @Override
  public void periodic() {
    // double distanceInches = vexUltrasonic.getRangeInches();
    // System.out.println(distanceInches);
    SmartDashboard.putNumber("Ultra Distance", getUltraDistance());
    SmartDashboard.putBoolean("shouldGrab?", getAtDistance());
    

    // This method will be called once per scheduler run
    // double distanceInches = vexUltrasonic.getRangeInches();
    // SmartDashboard.putNumber("Ping Distance", distanceInches);
  }
}