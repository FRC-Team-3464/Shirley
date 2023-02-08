// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class RunElevator extends CommandBase {
  /** Creates a new RunElevator. */
  
  private final ElevatorSubsystem elevatorSub;
  
  private final PIDController extendPID = new PIDController(0.02272727272, 0, 0);
  private double setpoint;
  private double speed;


  public RunElevator(ElevatorSubsystem elevatorSubsystem, double target) {
    setpoint = target;
    elevatorSub = elevatorSubsystem;
    addRequirements(elevatorSub);
    extendPID.setSetpoint(setpoint); // Set the setpoint to be whatever is passed
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed =  extendPID.calculate(elevatorSub.getElevatorPosition()); // Calculate the speed outputed based on a PID calculation given the current error. 
    elevatorSub.elevatorUp(speed);

    SmartDashboard.putNumber("Elevator Setpoint", setpoint);
    SmartDashboard.putNumber("Elevator Speed", speed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
