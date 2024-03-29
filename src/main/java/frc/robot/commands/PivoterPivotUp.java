// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivoterSubsystem;



// Stolen for debug. 



public class PivoterPivotUp extends CommandBase {
  /** Creates a new ArmPivotCommand. */

  private final PivoterSubsystem pivoterSub;
  // private double setpoint;// Is target position. 

  public PivoterPivotUp(PivoterSubsystem PivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    // Makes variables with the same values as the others in order to use them later in the Command
    // setpoint = target;
    pivoterSub = PivotSubsystem;
    addRequirements(PivotSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivoterSub.pivot(0.3); // What the heck?
    System.out.println("Pivoter " + pivoterSub.getPivoterRotation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivoterSub.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Makes sure that the position is where you want it to be at
    return false;
  }
}
