// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderReactLimit extends CommandBase {
  /** Creates a new ExtenderReactLimit. */
  private final ExtenderSubsystem extenderSub;

  public ExtenderReactLimit(ExtenderSubsystem extenderSubsystem) {
    extenderSub = extenderSubsystem;

    addRequirements(extenderSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    extenderSub.retract();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extenderSub.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return extenderSub.getMinSwitch();
  }
}
