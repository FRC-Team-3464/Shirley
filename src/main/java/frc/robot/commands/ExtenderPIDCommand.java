// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ExtenderConstants;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderPIDCommand extends CommandBase {
  /** Creates a new RunExtender. */
  
  private final ExtenderSubsystem extenderSub;
  private final PIDController extendPID = new PIDController(0.02272727272, 0, 0); // Verify using SysID
  private double setpoint;
  private double speed;

  public ExtenderPIDCommand(ExtenderSubsystem extenderSubsystem, double target) {
    setpoint = target;
    extenderSub = extenderSubsystem;
    addRequirements(extenderSub);
    extendPID.setSetpoint(setpoint); // Set the setpoint to be whatever is passed
    extendPID.setTolerance(ExtenderConstants.kExtenderMotorPort); // Set the position torence to be between + or - 0.75 in
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed =  extendPID.calculate(extenderSub.getExtenderPosition()); // Calculate the speed outputed based on a PID calculation given the current error. 
    // If speed is greater than 1.0, constrain it to 1.  
    if(speed > 1){
      speed = 1;
    }
    // Move the extender at that speed. 
    extenderSub.translateExtender(speed);

    SmartDashboard.putNumber("Extender Setpoint", setpoint);
    SmartDashboard.putNumber("Extender Speed", speed);
    SmartDashboard.putNumber("Extender Error", extendPID.getPositionError());
    SmartDashboard.putBoolean("Extender Finished", extendPID.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extenderSub.translateExtender(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(extendPID.atSetpoint()){ // Stop the command if we've reached the setpoint. 
      return true;
    }else{
      return false;
    }
  }
}
