// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSetPositionCMD extends CommandBase {
  /** Creates a new ElevatorSetPositionCMD. */

  private final ElevatorSubsystem elevatorSub;
  private double setpoint;

  public ElevatorSetPositionCMD(ElevatorSubsystem elevatorSubsystem, double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    setpoint = target;
    elevatorSub = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSub.elevatorUp(0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSub.elevatorUp(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(elevatorSub.getElevatorInchPosition() != setpoint){
      return false;
    }else{
      return true;
    }
  }
}
