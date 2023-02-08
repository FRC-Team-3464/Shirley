// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class RunElevator extends CommandBase {
  /** Creates a new RunElevator. */
  
  private final ElevatorSubsystem elevatorSub;
  
  private final PIDController extendPID = new PIDController(0.02272727272, 0, 0);
  private double setpoint;

  public RunElevator(ElevatorSubsystem elevatorSubsystem, double target) {
    setpoint = target;
    elevatorSub = elevatorSubsystem;
    addRequirements(elevatorSub);
    
    extendPID.setSetpoint(target);
    // extendPID.setTolerance(target);
    // extendPID.setTolerance(0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // elevatorSub.elevatorUp(extendPID.calculate());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
