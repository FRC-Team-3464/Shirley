// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class RunRollyGrabber extends CommandBase {
  /** Creates a new RunRollyGrabber. */
  private final LEDSubsystem ledSub;
  private final IntakeSubsystem intakeSub;
  public RunRollyGrabber(IntakeSubsystem intakeSub, LEDSubsystem ledSub) {
    this.intakeSub = intakeSub;
    this.ledSub = ledSub;
    addRequirements(intakeSub);
    addRequirements(ledSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.runIntake(0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
