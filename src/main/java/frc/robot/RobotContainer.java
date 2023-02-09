// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ElevatorPIDCMD;
import frc.robot.commands.ElevatorSetPositionCMD;
import frc.robot.commands.SimpleCommand;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here
  private final UltrasonicSubsystem ultrasonicSub = new UltrasonicSubsystem();
  private final ColorSensorSubsystem colorSub = new ColorSensorSubsystem();
  private final ElevatorSubsystem elevatorSub = new ElevatorSubsystem();
  private final DriveTrainSubsystem arcadeDriveSub = new DriveTrainSubsystem();


  // Commands defined here
  private final ElevatorPIDCMD PIDElevator = new ElevatorPIDCMD(elevatorSub, 22); // We want to get it to 22 inches. 
  private final ElevatorSetPositionCMD setElevator = new ElevatorSetPositionCMD(elevatorSub, 22);
  private final SimpleCommand simp = new SimpleCommand(elevatorSub, 0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
      OI.triggerAux.onTrue(new InstantCommand(elevatorSub::resetElevatorEncoder, elevatorSub));
      OI.button10Aux.onTrue(PIDElevator);
      OI.button11Aux.onTrue(setElevator);
      OI.button3Aux.onTrue(simp); // Thanks camden
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
