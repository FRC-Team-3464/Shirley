// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivoterSubsystem;

public class PivoterPIDCommand extends CommandBase {
  /** Creates a new SetGrabberCommand. */
  private final PivoterSubsystem pivoterSub;

  // private final PIDController armPIDController = new PIDController(0.00555555555, 0, 0);
  private final PIDController armPIDController = new PIDController(0.00455555555, 0, 0);
  private double speed;

  public PivoterPIDCommand(PivoterSubsystem pivoterSubsystem, double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    pivoterSub = pivoterSubsystem;
    addRequirements(pivoterSubsystem);
    armPIDController.setSetpoint(target);
    armPIDController.setTolerance(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = armPIDController.calculate(pivoterSub.getPivoterDegrees()); // This should help. 
    if(speed > 0.35){
      speed = 0.35;
    }
    pivoterSub.pivot(speed); // This doesn't seem to be working
    SmartDashboard.putNumber("Pivoter Error", armPIDController.getPositionError());
    SmartDashboard.putBoolean("Pivot Command Complete:" , armPIDController.atSetpoint());
    SmartDashboard.putNumber("Pivot Speed", speed);
    
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
