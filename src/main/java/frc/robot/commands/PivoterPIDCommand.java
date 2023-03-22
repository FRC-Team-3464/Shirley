// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivoterSubsystem;

public class PivoterPIDCommand extends CommandBase {
  private final PivoterSubsystem pivoterSub;
  private final PIDController pivotPIDController = new PIDController(0.01111111111, 0, 0); // SysID will update the value. 
  private double speed;

  public PivoterPIDCommand(PivoterSubsystem pivoterSubsystem, double target) {
    pivoterSub = pivoterSubsystem;
    pivotPIDController.setSetpoint(target);
    pivotPIDController.setTolerance(15); // Change tolarance // HELP this will be wrong. 
    addRequirements(pivoterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = pivotPIDController.calculate(pivoterSub.getPivoterDegrees()); // This should help. 
    // Constrain the speed to be 0.35 speed. 
    if(speed > 0.35){
      speed = 0.35;
    }

    pivoterSub.pivot(speed); // This doesn't seem to be working
    SmartDashboard.putNumber("Pivoter Error", pivotPIDController.getPositionError());
    SmartDashboard.putNumber("Output", speed);
    SmartDashboard.putBoolean("Pivot Command Complete:" , pivotPIDController.atSetpoint());    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if(pivotPIDController.atSetpoint()){
      return true;
    } else {
      return false;
    }
  }
}
