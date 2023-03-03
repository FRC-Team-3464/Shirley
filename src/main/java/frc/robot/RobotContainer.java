// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  // private final UltrasonicSubsystem ultrasonicSub = new UltrasonicSubsystem();
  private final ExtenderSubsystem extenderSub = new ExtenderSubsystem();
  private final PivoterSubsystem pivoterSub = new PivoterSubsystem();
  private final DrivetrainSubsystem driveSub = new DrivetrainSubsystem();
  private final GrabberSubsystem grabberSub = new GrabberSubsystem();
  private final PhotonVisionSubsystem photonSub = new PhotonVisionSubsystem();
  
  // Commands defined here
  private final ArcadeDriveCommand arcadeDriveCmd = new ArcadeDriveCommand(driveSub);

  /*
   * Pivoter Commands
   */

  // Pivoter Manual Speed Commands
  // private final Command pivotSpeedDown = pivoterSub.pivotManual(-0.125);
  // private final Command pivotSpeedUp = pivoterSub.pivotManual(0.25); // WHY is it inversed

  // Pivot till we hit the minimum limit switch. 
  private final FunctionalCommand pivotToMin = new FunctionalCommand(
    // what to do in initialize - basically nothing for us
    pivoterSub::stopMotor, // Fix
    // What to do during the command - run the motor unrestrained. 
    () -> pivoterSub.pivot(0.125), // Why is the motor inversed? It should be negative
    // When we finish the command. 
    (interrupted) -> pivoterSub.stopMotor(),
    // isFinished() - get switch value; when we hit the switch  
    pivoterSub::getSwitch,
    // Give us the requirements. 
    pivoterSub
  );

  /* 
  * Extender Commands
  */
  // Extender Manual Speed Commands
  private final Command extenderSpeedOut = extenderSub.translateManual(0.3);
  private final Command extenderSpeedIn = extenderSub.translateManual(-0.3);



  // ----------- DEBUG
  private final ExtenderExtendLimit extendExtender = new ExtenderExtendLimit(extenderSub);
  private final ExtenderReactLimit retractExtender = new ExtenderReactLimit(extenderSub);
  private final PivoterPivotMin pivotMin = new PivoterPivotMin(pivoterSub);
  private final PivoterSetCommand PivoterRotateUp = new PivoterSetCommand(pivoterSub);

  private final OpenGrabber openGrabber = new OpenGrabber(grabberSub);
  // private final 
  private final CloseGrabberCone grabCone = new CloseGrabberCone(grabberSub);
  private final CloseGrabberCube grabCube = new CloseGrabberCube(grabberSub);
  private final AddFeedFoward addFeedFoward = new AddFeedFoward(pivoterSub);
  private final InstantCommand limelightOff = new InstantCommand(photonSub::turnLEDOff, photonSub);
  private final InstantCommand limelightOn = new InstantCommand(photonSub::turnLEDOn, photonSub);



  // Store commands
  private final ExtenderReactLimit retractStore = new ExtenderReactLimit(extenderSub);
  private final PivoterPivotMin pivotStore = new PivoterPivotMin(pivoterSub);
  private final InstantCommand pivotEncoderReset = new InstantCommand(pivoterSub::resetEncoder, pivoterSub);
  private final InstantCommand extenderEncoderReset = new InstantCommand(extenderSub::resetExtenderEncoder, extenderSub);
  // private final InstantCommand manualForward = new InstantCommand(pivoterSub::pivotForward, pivoterSub);
  private final TargetCenterPIDCommand limeCenter = new TargetCenterPIDCommand(photonSub, driveSub, photonSub.getLimelightCamera());

  private final PivotToEncoderValue pivotToBottomPos = new PivotToEncoderValue(pivoterSub, 0);
  private final PivotToEncoderValue pivotToMiddlePos = new PivotToEncoderValue(pivoterSub, 0);
  private final PivotToEncoderValue pivotToTopPos = new PivotToEncoderValue(pivoterSub, 0);
  
  private final ExtenderExtendToEncoderValue extendToBottomPos = new ExtenderExtendToEncoderValue(extenderSub, 0);
  private final ExtenderExtendToEncoderValue extendToMiddlePos = new ExtenderExtendToEncoderValue(extenderSub, 0);
  private final ExtenderExtendToEncoderValue extendToTopPos = new ExtenderExtendToEncoderValue(extenderSub, 0);
  
  public final Command stowArm = new SequentialCommandGroup(retractStore, pivotStore,pivotEncoderReset,extenderEncoderReset);

  // Switch based translations
  // private final Command extendToMin = extenderSpeedIn.until(extenderSub::getMinSwitch); // Keep retracting till we hit the switch, then command ends. 
 
  // retract till we hit the miminimum. 
  private final FunctionalCommand retractToMin = new FunctionalCommand(
    // what to do in initialize - basically nothing for us
    extenderSub::stopMotor, // Stop the motor first
    // What to do during the command - run the motor unrestrained and retract the extender. . 
    () -> extenderSub.translateExtender(-0.3),
    // On finished command
    (interrupted) -> {extenderSub.stopMotor();
      System.out.println("Command Ended");},
    // isFinished() - get switch value.  
    extenderSub::getMinSwitch, // The minimum switch determines when we've ended. 
    // Give us the requirements. 
    extenderSub
  );

  // Extend till we hit the maximum. 
  private final FunctionalCommand extendToMax = new FunctionalCommand(
    // what to do in initialize - basically nothing for us
    extenderSub::stopMotor, 
    // What to do during the command - run the motor unrestrained. 
    () -> extenderSub.translateManual(0.3),
    // On finished command
    (interrupted) -> {extenderSub.stopMotor();
                      System.out.println("Command Ended");},
    // isFinished() - get switch value.  
    extenderSub::getMaxSwitch,
    // Give us the requirements. 
    extenderSub
  );


  /*
   * Grabber commands. 
   */

  private final FunctionalCommand grabberSetOpen = new FunctionalCommand(
    // what to do in initialize - basically nothing for us
    grabberSub::stopMotor, 
    // What to do during the command - run the motor unrestrained. 
    () -> grabberSub.runMotor(-0.125), // Counterclockwise closes. 
    // On finished command
    (interrupted) -> grabberSub.stopMotor(),
    // isFinished() - get switch value.  
    () -> (grabberSub.getGrabberDegrees() <= 0),
    // Give us the requirements. 
    grabberSub
  );

   
  private final FunctionalCommand grabberSetClosed = new FunctionalCommand(
    // what to do in initialize - basically nothing for us
    grabberSub::stopMotor, 
    // What to do during the command - run the motor unrestrained. 
    () -> grabberSub.runMotor(0.125), // CLockwise closes
    // On finished command
    (interrupted) -> grabberSub.stopMotor(),
    // isFinished() - get switch value.  
    () -> (grabberSub.getGrabberDegrees() >= 120),
    // Give us the requirements. 
    grabberSub
  );

  // Retract extender to min, then pivot -> represents going to the stored position. 
  // private final SequentialCommandGroup goToStorePosition = new SequentialCommandGroup(retractToMin, pivotToMin);

  
  // private final PivoterPIDCommand PIDPivotForward = new PivoterPIDCommand(pivoterSub, 45); //It's about that - please test
  // private final PivoterPIDCommand PIDPivotBack = new PivoterPIDCommand(pivoterSub, 0); // Dimension is wrong!!! 

  // private final PivoterSetCommand PivoterRotateForward = new PivoterSetCommand(pivoterSub, 45);

  

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
      CommandScheduler.getInstance().setDefaultCommand(pivoterSub, addFeedFoward);
      CommandScheduler.getInstance().setDefaultCommand(driveSub, arcadeDriveCmd); // Set the default command to have the robot always drive
      // CommandScheduler.getInstance().setDefaultCommand(extenderSub, new InstantCommand(extenderSub, ));
      // Extender Executables
      // OI.button10Aux.onTrue(new InstantCommand(grabberSub::resetGrabberDistance));
      OI.povButtonLeft.whileTrue(retractExtender);
      // OI.povButtonLeft.onFalse(extenderSub.commandStop());
      // OI.povButtonLeft.whileFalse(extenderSub::stopMotor);
      OI.povButtonRight.whileTrue(extendExtender);
      // OI.povButtonRight.onFalse(extenderSub.commandStop());
      // Pivoter Commands
      OI.povButtonDown.whileTrue(pivotMin);
      OI.povButtonUp.whileTrue(PivoterRotateUp);

      OI.button2Aux.onTrue(stowArm);

      // Grabber Commands

      OI.button6Aux.whileTrue(grabCone);
      // OI.button4Aux.toggleOnTrue(grabCube);
      OI.button3Aux.whileTrue(openGrabber);

      OI.button11Aux.onTrue(limelightOff);
      OI.button12Aux.onTrue(limelightOn);

      OI.buttonY.whileTrue(limeCenter);
      
      
    

      // // Switich based commands
      // OI.button3Aux.onTrue(retractToMin);
      // OI.button4Aux.onTrue(extendToMax);

      // OI.button5Aux.onTrue(pivotToMin);
      
      // // Open Grabber Set command. 
      // // OI.button6Aux.onTrue(extendToMax);
      // OI.button6Aux.whileTrue(grabberSpeedClose);
      // OI.button7Aux.whileTrue(grabberSpeedOpen); // That will be the default. 

      // Retract pivoter to min, rotate extender to min. 
      //OI.button8Aux.onTrue(testGo); //THIS IS A VERY DANGEROUS LINE




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
     // runs StowArm command once when robot is initialized
     CommandScheduler.getInstance().schedule(stowArm);

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
