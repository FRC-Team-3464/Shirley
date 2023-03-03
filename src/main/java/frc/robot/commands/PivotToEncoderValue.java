// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivoterSubsystem;


public class PivotToEncoderValue extends CommandBase {
  /** Creates a new PivotForward. */
  private final PivoterSubsystem pivotSub;
  private final double setPoint;
  public PivotToEncoderValue(PivoterSubsystem pivotSub, double target) {
    this.pivotSub = pivotSub;
    setPoint = target;
    addRequirements(pivotSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotSub.pivot(1);
    // System.out.println("Pivoter " + pivotSub.getPivoterRotation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSub.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pivotSub.getPivoterRotation() > setPoint){
      return true;
    }
    return false;
  }
}
