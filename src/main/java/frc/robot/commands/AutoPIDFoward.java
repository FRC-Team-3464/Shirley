// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoPIDFoward extends CommandBase {
  /** Creates a new AutoDriveFoward. */

  private final DrivetrainSubsystem driveSub;
  // private double target;
  private PIDController forwardController = new PIDController(2.6512, 0, 0.41464);

  public AutoPIDFoward(DrivetrainSubsystem driveSub, double target) { // This will most likely be in inches. 
    this.driveSub = driveSub;
    // this.target = target; // Set the tarbet defined above to be what is passed into target. 
    forwardController.setSetpoint(target);
    // forwardController.setTolerance(target);
    // Make sure that we require the subsystem for this command to work. 
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSub.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSub.arcadeDrive(forwardController.calculate(-driveSub.getLeftPosition()), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.stopDrive();
    driveSub.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //  Become smaller value until it's less than the target 
    // return (driveSub.getLeftPosition()) < target; //  until encoder value < -50
    return forwardController.atSetpoint();
  }
}
