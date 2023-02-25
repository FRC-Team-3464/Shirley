// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class GrabberSetCommand extends CommandBase {
  private final GrabberSubsystem grabberSub;
  private boolean close;
  // Set the grabber to open/close by rotating 120 degrees counter/clockwise.

  public GrabberSetCommand(GrabberSubsystem grabberSubsystem, boolean isClose) {
    close = isClose;
    grabberSub = grabberSubsystem;
    addRequirements(grabberSubsystem);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(close){
      grabberSub.runMotor(0.25); // Counterclockwise closes the grabber. 
    }else{
      grabberSub.runMotor(-0.25); // Counterclockwise upens the grabbery. 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    grabberSub.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((close && grabberSub.getGrabberDegrees() >= 120) || (!close && grabberSub.getGrabberDegrees() <= 0)){
      // We wanted it to close and it's greater than or equal to 120 degrees or it's open and the grabber degrees is less than or equal to 0
      return true; // End the command
    }else{
      return false;
    }
   
  }
}