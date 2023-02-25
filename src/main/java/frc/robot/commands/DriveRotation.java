// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;

public class DriveRotation extends PIDCommand {
  
    //private final DrivetrainSubsystem driveSub;
    //private final double setpoint;
  
    public DriveRotation(double targetDistance, DrivetrainSubsystem drive) {
        super(
            drive.getController(),
            // Close loop on heading
            drive::getMeasurement,
            // Set reference to target
            targetDistance,
            // Pipe output to turn robot
            output -> drive.arcadeDrive(0, output),
            // Require the drive
            drive);
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
     
   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
    return getController().atSetpoint();
   }

}
