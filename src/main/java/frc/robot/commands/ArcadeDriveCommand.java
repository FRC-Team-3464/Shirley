// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.DrivetrainRamp;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ArcadeDriveCommand extends CommandBase {
  /** Creates a new ArcadeDriveCommand. */
  
  private final XboxController controller = OI.xBoxController;
  private final DrivetrainSubsystem arcadeDriveSub;
  private final DrivetrainRamp driveRamp;
  
  public ArcadeDriveCommand(DrivetrainSubsystem driveTrainSub, DrivetrainRamp drivetrainRamp ) {
    arcadeDriveSub = driveTrainSub;
    driveRamp = drivetrainRamp;

    addRequirements(drivetrainRamp); // Require the ramp function. 
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
    // Uses the left joystick X and Y values, and uses 80% of the value
    // Ramp the drivetrain. 
    arcadeDriveSub.arcadeDrive(driveRamp.applyAsDouble((controller.getLeftY() * 0.5)) , controller.getRightX() * 0.6); // Halve the speed - we don't need to go so fast now. 
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