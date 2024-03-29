// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OI;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;

public class AutoFeederDistance extends CommandBase {
  /** Creates a new AutoFeederDistance. */
  private final XboxController controller = OI.xBoxController;
 
  private final DrivetrainSubsystem drivetrain;
  private final UltrasonicSubsystem ultrasonic;
  private final LEDSubsystem led;
  private double target;
  // private final PIDController autoFeederPID = new PIDController(0.0285, 0.00, 0);


  public AutoFeederDistance(DrivetrainSubsystem drivetrainSub, UltrasonicSubsystem ultrasonicSub, LEDSubsystem ledSub, double target) {
    drivetrain = drivetrainSub;
    ultrasonic = ultrasonicSub;
    led = ledSub;
    this.target = target;
    addRequirements(drivetrain, ultrasonic, led);
    // autoFeederPID.setSetpoint(target);
    // autoFeederPID.setTolerance(1);
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    led.red();
    // drivetrain.arcadeDrive(-autoFeederPID.calculate(ultrasonic.getUltraDistance()), 0.3 * controller.getRightX());
    drivetrain.arcadeDrive(-0.2, 0.3 * controller.getRightX());

    // SmartDashboard.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if(!interrupted){
    // controller.setRumble(RumbleType.kBothRumble, 0.5);
    led.green(); // If command ends without it being inturrupted, turn LED green. 
    // }
    drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return autoFeederPID.atSetpoint();
    return ultrasonic.getUltraDistance() <= target; // Going in decreases distance, so when sensor is smaller than actual 
    // ultrasonic.getUltraDistance() <= target
  }
}
