// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.PivoterPIDCommand;
import frc.robot.commands.ExtenderPIDCommand;
import frc.robot.commands.ExtenderSetPositionCommand;
import frc.robot.commands.GrabberSetCommand;
import frc.robot.subsystems.PivoterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
  private final ExtenderSubsystem extenderSub = new ExtenderSubsystem();
  private final PivoterSubsystem pivoterSub = new PivoterSubsystem();
  private final DrivetrainSubsystem driveSub = new DrivetrainSubsystem();
  private final GrabberSubsystem grabberSub = new GrabberSubsystem();
  // Commands defined here
  private final ArcadeDriveCommand arcadeDriveCmd = new ArcadeDriveCommand(driveSub);
  private final PivoterPIDCommand PIDArmForward = new PivoterPIDCommand(pivoterSub, 90);
  private final PivoterPIDCommand PIDArmBack = new PivoterPIDCommand(pivoterSub, 0); // Dimension is wrong!!! 

  private final ExtenderPIDCommand PIDExtenderExtend = new ExtenderPIDCommand(extenderSub, 22); // We want to get it to 22 inches. 
  private final ExtenderPIDCommand PIDExtenderRetract = new ExtenderPIDCommand(extenderSub, 0); // We want to get it to 22 inches. 
 
  // Alternate forms - use in test
  private final ExtenderSetPositionCommand noPIDCmdExtenderExtend = new ExtenderSetPositionCommand(extenderSub, 22);
  private final ExtenderSetPositionCommand noPIDCmdExtenderRetract = new ExtenderSetPositionCommand(extenderSub, 0);
  

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true` hmmmm...
      
      // Run default command as the arcade drive command.
      CommandScheduler.getInstance().setDefaultCommand(driveSub, arcadeDriveCmd); // Set the default command to have the robot always drive
      
      // Trigger command execution. 
      OI.triggerAux.toggleOnTrue(openGrabber);
      OI.triggerAux.toggleOnFalse(closeGrabber);

      OI.button5Aux.toggleOnTrue(PIDArmForward);
      OI.button5Aux.toggleOnFalse(PIDArmBack);
 
      OI.button7Aux.toggleOnTrue(PIDExtenderExtend);
      OI.button7Aux.toggleOnFalse(PIDExtenderRetract);
      
      // OI.button9Aux.toggleOnTrue(noPIDCmdExtenderExtend); // Don't think we need this
      // OI.button9Aux.toggleOnFalse(noPIDCmdExtenderRetract); // Don't think we need this

      // OI.button11Aux.toggleOnTrue(simpleSetExtenderExtend); // Uncomment to test
      // There is no code for simpleSetExtenderRetract
      
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
