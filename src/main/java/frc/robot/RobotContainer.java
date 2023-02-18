// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.ArmPIDCommand;
import frc.robot.commands.ElevatorPIDCMD;
import frc.robot.commands.ElevatorSetPositionCMD;
import frc.robot.commands.ElevatorSimpleSetPositionCommand;
import frc.robot.commands.GrabberSetCommand;
import frc.robot.subsystems.ArmPivoterSubsystem;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
  private final ArmPivoterSubsystem pivoterSub = new ArmPivoterSubsystem();
  private final DrivetrainSubsystem driveSub = new DrivetrainSubsystem();
  private final GrabberSubsystem grabberSub = new GrabberSubsystem();
  // Commands defined here
  private final ArcadeDriveCommand arcadeDriveCmd = new ArcadeDriveCommand(driveSub);
  private final ArmPIDCommand PIDArm = new ArmPIDCommand(pivoterSub, 90);
  private final ArmPIDCommand PIDArmBack = new ArmPIDCommand(pivoterSub, 0);  
  private final ElevatorPIDCMD PIDElevator = new ElevatorPIDCMD(elevatorSub, 22); // We want to get it to 22 inches. 
  private final ElevatorSetPositionCMD noPIDCmdElevator = new ElevatorSetPositionCMD(elevatorSub, 22);
  private final ElevatorSimpleSetPositionCommand simpleSetElevator = new ElevatorSimpleSetPositionCommand(elevatorSub, 0); // You better change this. 
  private final GrabberSetCommand openGrabber = new GrabberSetCommand(grabberSub, true);
  private final GrabberSetCommand closeGrabber = new GrabberSetCommand(grabberSub, false);

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
      
      CommandScheduler.getInstance().setDefaultCommand(driveSub, arcadeDriveCmd); // Set the default command to have the robot always drive

      OI.triggerAux.toggleOnTrue(openGrabber);
      OI.triggerAux.toggleOnFalse(closeGrabber);
      // OI.triggerAux.onTrue(new InstantCommand(elevatorSub::resetElevatorEncoder, elevatorSub));
      OI.button10Aux.onTrue(PIDElevator);
      OI.button11Aux.onTrue(noPIDCmdElevator); // Don't think we need this
      OI.button3Aux.onTrue(simpleSetElevator); // Thanks camden

      // OI.button12Aux.onTrue(new InstantCommand(elevatorSub))
      OI.button5Aux.onTrue(PIDArm);
      OI.button6Aux.onTrue(PIDArmBack);
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
