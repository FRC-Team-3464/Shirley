// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ArcadeDriveCommand extends CommandBase {
  /** Creates a new ArcadeDriveCommand. */
  
  private final XboxController controller = new XboxController(3);
  private final DriveTrainSubsystem driveSub;
  
  
  public ArcadeDriveCommand(DriveTrainSubsystem driveTrainSub) {
    driveSub = driveTrainSub;
    addRequirements(driveTrainSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uses the left joystick X and Y values 
    driveSub.arcadeDrive(controller.getLeftY() * 0.8, controller.getLeftX());
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
