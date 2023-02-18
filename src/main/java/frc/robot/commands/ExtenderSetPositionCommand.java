// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderSetPositionCommand extends CommandBase {
  /** Creates a new ElevatorSetPositionCMD. */

  private final ExtenderSubsystem extenderSub;
  private double setpoint;

  public ExtenderSetPositionCommand(ExtenderSubsystem extenderSubsystem, double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    setpoint = target;
    extenderSub = extenderSubsystem;
    addRequirements(extenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    extenderSub.elevatorUp(0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extenderSub.elevatorUp(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(extenderSub.getElevatorInchPosition() >= setpoint){
      return false;
    }else{
      return true;
    }
  }
}
