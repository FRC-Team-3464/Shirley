// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSimpleSetPositionCommand extends CommandBase {
  /** Creates a new RunElevator. */
  
  private final ElevatorSubsystem elevatorSub;
  
  private final PIDController extendPID = new PIDController(0.02272727272, 0, 0);
  private double setpoint;
  private double speed;


  public ElevatorSimpleSetPositionCommand(ElevatorSubsystem elevatorSubsystem, double target) {
    setpoint = target;
    elevatorSub = elevatorSubsystem;
    addRequirements(elevatorSub);
    extendPID.setSetpoint(setpoint); // Set the setpoint to be whatever is passed
    extendPID.setTolerance(0.75); // Set the position torence to be between + or - 0.75 in
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSub.elevatorUp(0);
    elevatorSub.resetElevatorEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*speed =  extendPID.calculate(elevatorSub.getElevatorPosition()); // Calculate the speed outputed based on a PID calculation given the current error. 
    elevatorSub.elevatorUp(speed);*/
    elevatorSub.regularSetPoint();
    SmartDashboard.putNumber("Elevator Setpoint", setpoint);
    SmartDashboard.putNumber("Elevator Speed", speed);
    SmartDashboard.putNumber("Error", extendPID.getPositionError());
    // SmartDashboard.putNumber(""), setpoint)
    SmartDashboard.putBoolean("PID Command Finished", extendPID.atSetpoint());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSub.elevatorUp(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //if(extendPID.atSetpoint()){
      //return true;
    //}else{
      //return false;
    //}
  }
}

