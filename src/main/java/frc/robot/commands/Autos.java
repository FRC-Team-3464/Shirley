// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
// HUH?
// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // private final AutoDriveFoward autoForward;
  
  // Auto Command to go forward and balance. 
  public static CommandBase BalanceOnly(AutoDriveFoward forward, BalanceDistance distance, BalanceHold holdDrivetrain) {
  //  autoForward = forward;
    return new SequentialCommandGroup(forward, distance, holdDrivetrain);
  }

  

  // 

  public Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
