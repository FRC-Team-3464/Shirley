// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class GrabberSetCommand extends CommandBase {
  /** Creates a new GrabberCommand. */

  private final GrabberSubsystem grabberSub;
  private boolean open;
  // In this command, we want to set the grabber to open/close by rotating 120 degrees counter/clockwise.



  public GrabberSetCommand(GrabberSubsystem grabberSubsystem, boolean isOpen) {
    // Use addRequirements() here to declare subsystem dependencies.
    open = isOpen;
    grabberSub = grabberSubsystem;
    addRequirements(grabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // grabberSub.resetGrabberDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(open){
      grabberSub.runMotor(0.25);
    }else{
      grabberSub.runMotor(-0.25);
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

    if((open && grabberSub.getGrabberDegrees() >= 120) || (!open && grabberSub.getGrabberDegrees() <= 0)){
      return true;
    }else{
      return false;
    }
   
  }
}