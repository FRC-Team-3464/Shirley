// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
public class AutoPIDFoward extends PIDCommand {
    public AutoPIDFoward(double targetDistance, DrivetrainSubsystem drive) {
        super(
            drive.getForwardController(), // Get the PID Controller for driving forward. 
            // Close loop on heading
            drive::getLeftPosition, // Get our encoder's left position
            // Set reference to target
            targetDistance,
            // Pipe output to move robot forward
            output -> drive.arcadeDrive(MathUtil.clamp(output, -0.75, 0.75), 0), // Clamp output to 75% speed. 
            // Require the drive
            drive);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    getController().reset(); // We want to reset the controller before using it.
  }

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
