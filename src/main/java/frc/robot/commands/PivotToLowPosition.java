// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.PivoterConstants;
import frc.robot.subsystems.PivoterSubsystem;

public class PivotToLowPosition extends CommandBase {
  /** Creates a new PivotToHighPosition. */
  private final PivoterSubsystem pivoterSub;
  private final double setpoint;
  public PivotToLowPosition(PivoterSubsystem pivoterSub, double target) {
    this.pivoterSub = pivoterSub;
    setpoint = target;
    addRequirements(pivoterSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivoterSub.pivot(-0.1); // set to max speed. 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //pivoterSub.addFeedFoward();
    // pivoterSub.re
    pivoterSub.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pivoterSub.getSwitch()){
      return true;
    }
    return (pivoterSub.getPivoterRotation() <= setpoint);
  }
}
