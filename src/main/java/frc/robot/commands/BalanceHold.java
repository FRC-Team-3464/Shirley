// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BalanceHoldPIDSubsystem;
// import frc.robot.subsystems.EncoderSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalanceHold extends CommandBase {
 
  
  private final BalanceHoldPIDSubsystem holdSub;
  private final DrivetrainSubsystem driveSub;
  /** Creates a new BalanceHold. */
  public BalanceHold(BalanceHoldPIDSubsystem balance, DrivetrainSubsystem driveSub) {
    holdSub = balance;
    this.driveSub = driveSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(holdSub, driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSub.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    holdSub.useOutput(holdSub.getMeasurement(), 0); // Run the autobalance PID Command
    System.out.println("Running Command AutoHold PID");
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
