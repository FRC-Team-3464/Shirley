// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TankDriveCommand extends CommandBase {
  /** Creates a new TankDriveCommand. */
  
  private final DriveTrainSubsystem tankDriveSub; //Leave blank for now

  public TankDriveCommand(DriveTrainSubsystem driveTrainSub) {
    tankDriveSub = driveTrainSub;
    addRequirements(tankDriveSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Gets 80% of values of the two sticks to create a tank drive
    tankDriveSub.driveTank(OI.leftStick.getY() * 0.8, OI.rightStick.getY() * 0.8);
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
