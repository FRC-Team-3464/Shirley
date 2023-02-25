// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderSetPositionCommand extends CommandBase {
  private final ExtenderSubsystem extenderSub;
  private double setpoint;
  private double speed;

  public ExtenderSetPositionCommand(ExtenderSubsystem extenderSubsystem, double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    setpoint = target;
    extenderSub = extenderSubsystem;
    addRequirements(extenderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(extenderSub.getExtenderInchPosition() < setpoint){ // If current position is less, we want it to be positive so it extends. 
      speed = 0.25;
    }else if(extenderSub.getExtenderInchPosition() > setpoint){ // If current position is greater than, speed is negative so we move counter clockwise. 
      speed = -0.25; // Rotate backwards, retract. 
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    extenderSub.translateExtender(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extenderSub.translateExtender(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(extenderSub.getExtenderInchPosition() - setpoint) > ExtenderConstants.tolerance){ // if the difference less than 0.75
      return true;
    }else{
      return false;
    }
  }
}
