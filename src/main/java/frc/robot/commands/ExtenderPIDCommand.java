// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderPIDCommand extends CommandBase {
  /** Creates a new RunElevator. */
  
  private final ExtenderSubsystem extenderSub;
  
  private final PIDController extendPID = new PIDController(0.02272727272, 0, 0);
  private double setpoint;
  private double speed;


  public ExtenderPIDCommand(ExtenderSubsystem elevatorSubsystem, double target) {
    setpoint = target;
    extenderSub = elevatorSubsystem;
    addRequirements(extenderSub);
    extendPID.setSetpoint(setpoint); // Set the setpoint to be whatever is passed
    extendPID.setTolerance(0.75); // Set the position torence to be between + or - 0.75 in
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed =  extendPID.calculate(extenderSub.getElevatorPosition()); // Calculate the speed outputed based on a PID calculation given the current error. 
    extenderSub.elevatorUp(speed);

    SmartDashboard.putNumber("Elevator Setpoint", setpoint);
    SmartDashboard.putNumber("Elevator Speed", speed);
    SmartDashboard.putNumber("Error", extendPID.getPositionError());
    // SmartDashboard.putNumber(""), setpoint)
    SmartDashboard.putBoolean("PID Command Finished", extendPID.atSetpoint());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extenderSub.elevatorUp(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(extendPID.atSetpoint()){
      return true;
    }else{
      return false;
    }
  }
}
