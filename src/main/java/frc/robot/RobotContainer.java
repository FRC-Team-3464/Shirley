// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.PivoterPIDCommand;
import frc.robot.commands.PivoterSetCommand;
import frc.robot.commands.TargetCenterAndRangePIDCommand;
import frc.robot.commands.TargetCenterPIDCommand;
import frc.robot.commands.TargetRangePIDCommand;
import frc.robot.commands.ExtenderPIDCommand;
import frc.robot.commands.ExtenderSetPositionCommand;
import frc.robot.commands.GrabberSetCommand;
import frc.robot.subsystems.PivoterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.UltrasonicSubsystem;

import java.util.Arrays;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.counter.ExternalDirectionCounter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 * 
 * 
 */
public class RobotContainer {
  // The robot's subsystems are defined here
  private final UltrasonicSubsystem ultrasonicSub = new UltrasonicSubsystem();
  private final ExtenderSubsystem extenderSub = new ExtenderSubsystem();
  private final PivoterSubsystem pivoterSub = new PivoterSubsystem();
  private final DrivetrainSubsystem driveSub = new DrivetrainSubsystem();
  private final GrabberSubsystem grabberSub = new GrabberSubsystem();
  private final PhotonVisionSubsystem photonSub = new PhotonVisionSubsystem();
  
  // Commands defined here
  private final ArcadeDriveCommand arcadeDriveCmd = new ArcadeDriveCommand(driveSub);

  // Pivoter Manual Speed Commands
  private final Command pivotSpeedDown = pivoterSub.pivotManual(0.125);
  private final Command pivotSpeedUp = pivoterSub.pivotManual(-0.5); // WHY is it inversed

  // Pivot till we hit the mimium. 
  private final FunctionalCommand pivotToMin = new FunctionalCommand(
    // what to do in initialize - basically nothing for us
    null, 
    // What to do during the command - run the motor unrestrained. 
    () -> pivoterSub.pivot(0.125), // Why is the number inversed?
    // On finished command
    (interrupted) -> pivoterSub.stopMotor(),
    // isFinished() - get switch value.  
    pivoterSub::getSwitch,
    // Give us the requirements. 
    pivoterSub
  );

  // Extender Manual Speed Commands
  private final Command extenderSpeedOut = extenderSub.translateManual(0.3);
  private final Command extenderSpeedIn = extenderSub.translateManual(-0.3);

  // Switch based translations
  // private final Command extendToMin = extenderSpeedIn.until(extenderSub::getMinSwitch); // Keep retracting till we hit the switch, then command ends. 
 
  // retract till we hit the miminimum. 
  private final FunctionalCommand retractToMin = new FunctionalCommand(
    // what to do in initialize - basically nothing for us
    null, 
    // What to do during the command - run the motor unrestrained and retract the extender. . 
    () -> extenderSub.translateExtender(-0.3),
    // On finished command
    (interrupted) -> extenderSub.stopMotor(), // When end, stop extender. 
    // isFinished() - get switch value.  
    extenderSub::getMinSwitch, // The minimum switch determines when we've ended. 
    // Give us the requirements. 
    extenderSub
  );

    
  private final FunctionalCommand extendToMax = new FunctionalCommand(
    // what to do in initialize - basically nothing for us
    null, 
    // What to do during the command - run the motor unrestrained. 
    () -> extenderSub.translateManual(0.3),
    // On finished command
    (interrupted) -> extenderSub.stopMotor(),
    // isFinished() - get switch value.  
    extenderSub::getMaxSwitch,
    // Give us the requirements. 
    extenderSub
  );

  private final FunctionalCommand grabberSetOpen = new FunctionalCommand(
    // what to do in initialize - basically nothing for us
    null, 
    // What to do during the command - run the motor unrestrained. 
    () -> grabberSub.runMotor(-0.125),
    // On finished command
    (interrupted) -> grabberSub.stopMotor(),
    // isFinished() - get switch value.  
    () -> (grabberSub.getGrabberDegrees() <= 0),
    // Give us the requirements. 
    grabberSub
  );

   
  private final FunctionalCommand grabberSetClosed = new FunctionalCommand(
    // what to do in initialize - basically nothing for us
    null, 
    // What to do during the command - run the motor unrestrained. 
    () -> grabberSub.runMotor(0.125),
    // On finished command
    (interrupted) -> grabberSub.stopMotor(),
    // isFinished() - get switch value.  
    () -> (grabberSub.getGrabberDegrees() >= 120),
    // Give us the requirements. 
    grabberSub
  );

 
  // Grabber Manual Speed Commands
  private final Command grabberSpeedClose = grabberSub.runMotor(0.125);
  private final Command grabberSpeedOpen = grabberSub.runMotor(-0.125);
  
  
  // private final PivoterPIDCommand PIDPivotForward = new PivoterPIDCommand(pivoterSub, 45); //It's about that - please test
  // private final PivoterPIDCommand PIDPivotBack = new PivoterPIDCommand(pivoterSub, 0); // Dimension is wrong!!! 

  // private final PivoterSetCommand PivoterRotateForward = new PivoterSetCommand(pivoterSub, 45);
  // private final PivoterSetCommand PivoterRotateBack = new PivoterSetCommand(pivoterSub, 0);
  

  // private final ExtenderPIDCommand PIDExtenderExtend = new ExtenderPIDCommand(extenderSub, 22); // We want to get it to 22 inches. 
  // private final ExtenderPIDCommand PIDExtenderRetract = new ExtenderPIDCommand(extenderSub, 0); // We want to get it to 22 inches. 
 
  // private final GrabberSetCommand openGrabber = new GrabberSetCommand(grabberSub, true);
  // private final GrabberSetCommand closeGrabber = new GrabberSetCommand(grabberSub, false);

  
  // // Alternate forms - use in test
  // private final PivoterSetCommand PivoterHighPoint = new PivoterSetCommand(pivoterSub, 45);
  // private final PivoterSetCommand PivoterLowPoint = new PivoterSetCommand(pivoterSub, 0);
  
  // private final ExtenderSetPositionCommand noPIDCmdExtenderExtend = new ExtenderSetPositionCommand(extenderSub, 22);
  // private final ExtenderSetPositionCommand noPIDCmdExtenderRetract = new ExtenderSetPositionCommand(extenderSub, 0);

  // // PID Aim Commands
  // // Auto center and get in range with the limelight or the apriltag camera. 
  // private final TargetCenterAndRangePIDCommand limeCenterAndRange = new TargetCenterAndRangePIDCommand(photonSub, driveSub, photonSub.getLimelightCamera());
  // private final TargetCenterAndRangePIDCommand aprilCenterAndRange = new TargetCenterAndRangePIDCommand(photonSub, driveSub, photonSub.getColorCamera());

  // private final TargetCenterPIDCommand limeCenter = new TargetCenterPIDCommand(photonSub, driveSub, photonSub.getLimelightCamera());
  // private final TargetCenterPIDCommand aprilCenter = new TargetCenterPIDCommand(photonSub, driveSub, photonSub.getColorCamera());

  // private final TargetRangePIDCommand limeRange = new TargetRangePIDCommand(photonSub, driveSub, photonSub.getLimelightCamera());
  // private final TargetRangePIDCommand aprilRange = new TargetRangePIDCommand(photonSub, driveSub, photonSub.getColorCamera());
 

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
      // Run default command as the arcade drive command.
      CommandScheduler.getInstance().setDefaultCommand(driveSub, arcadeDriveCmd); // Set the default command to have the robot always drive
      
      // Extender Executables
      OI.buttonRB.whileTrue(extenderSpeedIn);
      OI.buttonLB.whileTrue(extenderSpeedOut);

      // Pivoter Commands
      OI.buttonB.whileTrue(pivotSpeedDown);
      OI.buttonY.whileTrue(pivotSpeedUp);

      // Grabber Commands
      OI.triggerAux.onTrue(grabberSetClosed);
      OI.button2Aux.onTrue(grabberSetOpen);
      

      // Switich based commands
      OI.button3Aux.onTrue(retractToMin);
      OI.button4Aux.onTrue(extendToMax);

      OI.button5Aux.onTrue(pivotToMin);
      
      // Open Grabber Set command. 
      // OI.button6Aux.onTrue(extendToMax);
      OI.button6Aux.whileTrue(grabberSpeedClose);
      OI.button7Aux.whileTrue(grabberSpeedOpen); // That will be the default. 

      

      

      

      // Trigger command execution.
      // OI.triggerAux.toggleOnTrue(openGrabber);
      // OI.triggerAux.toggleOnFalse(closeGrabber);


      // // OI.button5Aux.toggleOnTrue(PIDPivotForward);
      // // OI.button5Aux.toggleOnFalse(PIDPivotBack);

      // OI.button5Aux.toggleOnTrue(PivoterHighPoint);
      // OI.button5Aux.toggleOnFalse(PivoterLowPoint);

      // OI.button9Aux.toggleOnTrue(noPIDCmdExtenderExtend); // Don't think we need this
      // OI.button9Aux.toggleOnFalse(noPIDCmdExtenderRetract); // Don't think we need this

      
      // // OI.button7Aux.toggleOnTrue(PIDExtenderExtend);
      // // OI.button7Aux.toggleOnFalse(PIDExtenderRetract);
      
  
      // OI.buttonX.whileTrue(limeCenterAndRange);
      // OI.buttonA.whileTrue(aprilCenterAndRange);


    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
    // config.setKinematics(driveSub.getKinematics()); // Set the kinematics to be the one from drivesub.
    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())), config); // Move forward to 1.0, which is one meter forward. 
    // RamseteCommand command = new RamseteCommand(trajectory, driveSub::getPose, new RamseteController(2.0, 0.7), driveSub.getFeedforward(), driveSub.getKinematics(), driveSub::getSpeeds, driveSub.getLeftPIDController(), driveSub.getRightPIDController(), driveSub::setVolts, driveSub);
    // return command; 
    return null;
  }
}
