// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class ReverseRollyGrabber extends CommandBase {
  /** Creates a new ReverseRollyGrabber. */
  public final IntakeSubsystem intakeSub;
  public final LEDSubsystem ledSub;
  public ReverseRollyGrabber(IntakeSubsystem intakeSub, LEDSubsystem ledSub) {
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
    if(ledSub.getLEDState() == "cube") {
      intakeSub.runIntake(-0.25);
    }
    else if(ledSub.getLEDState() == "cone") {
      intakeSub.runIntake(-0.15);
    }
    // in case he forgets to press, hopefully won't use this tho
    else {
      intakeSub.runIntake(-0.20);
    }
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
