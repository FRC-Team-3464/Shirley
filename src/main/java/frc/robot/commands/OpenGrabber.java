// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class OpenGrabber extends CommandBase {
  /** Creates a new OpenGrabber. */
  private final GrabberSubsystem grabberSub;
  public OpenGrabber(GrabberSubsystem grabberSub) {
    this.grabberSub = grabberSub;
    addRequirements(grabberSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    grabberSub.runMotor(0.15);
    //System.out.println(grabberSub.getGrabberDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    grabberSub.runMotor(0.05); // Add the feedforward for the grabber to hold open. 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(grabberSub.getGrabberDegrees() >= 1100){
      return true;
    }
    return false;
  }
}
