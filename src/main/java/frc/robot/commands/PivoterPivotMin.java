// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivoterSubsystem;

public class PivoterPivotMin extends CommandBase {
  /** Creates a new PivoterPivotMin. */
  private final PivoterSubsystem pivoterSub;
  public PivoterPivotMin(PivoterSubsystem pivoterSub) {
    this.pivoterSub = pivoterSub;
    addRequirements(pivoterSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivoterSub.pivot(-0.2);
    // System.out.println("Pivoter " + pivoterSub.getPivoterRotation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("End");
    pivoterSub.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivoterSub.getSwitch();
  }
}
