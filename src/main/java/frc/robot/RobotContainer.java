// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.PivoterConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

// import java.util.Arrays;
// import edu.wpi.first.math.controller.RamseteController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.counter.ExternalDirectionCounter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
  private final DrivetrainRamp driveRamp = new DrivetrainRamp(1.33, 2.5); // These values may be wrong. 
  // private final PhotonVisionSubsystem photonSub = new PhotonVisionSubsystem(); // I just want to read the values in periodic().
  
  /*
   * Drivetrain Commands
   */

  private final ArcadeDriveCommand arcadeDriveCmd = new ArcadeDriveCommand(driveSub, driveRamp); // Add the drive ramp
  private final InstantCommand drivetrainEncoderReset = new InstantCommand(driveSub::resetEncoders, driveSub); 

  /* 
   * Extender Commands
   */
 
  private final ExtenderExtendLimit extendExtender = new ExtenderExtendLimit(extenderSub); // Extend the extender till we reach the max limit. 
  private final ExtenderRetractLimit retractExtender = new ExtenderRetractLimit(extenderSub); // Retract the extender till we reach the min limit. 
  
  /*
   * Pivoter Commands
   */ 

  private final PivoterPivotUp PivoterRotateUp = new PivoterPivotUp(pivoterSub); // Rotate the pivoter up...indefinitly. 
  private final PivoterPivotMin pivotMin = new PivoterPivotMin(pivoterSub); // Rotate down till we rech the pivoter min switch. 
  private final AddFeedFoward addFeedFoward = new AddFeedFoward(pivoterSub); // Hold the pivoter up. 

  /*
   * Grabber Commands
   */ 
  private final OpenGrabber openGrabber = new OpenGrabber(grabberSub);
  private final CloseGrabberCone grabCone = new CloseGrabberCone(grabberSub);
  private final CloseGrabberCube grabCube = new CloseGrabberCube(grabberSub);

  /*
   * Store Commands: We need to create the commands again to follow the syntax of creating sequential commands. 
   */ 

  private final ExtenderRetractLimit retractStore = new ExtenderRetractLimit(extenderSub);
  private final PivoterPivotMin pivotStore = new PivoterPivotMin(pivoterSub);

  private final InstantCommand pivotEncoderReset = new InstantCommand(pivoterSub::resetEncoder, pivoterSub);
  private final InstantCommand extenderEncoderReset = new InstantCommand(extenderSub::resetExtenderEncoder, extenderSub);

  public final Command stowArm = new SequentialCommandGroup(retractStore, pivotStore, pivotEncoderReset, extenderEncoderReset);

  /*
   * Position based commands:
   * Set the pivoter/extender to certain setpoints. 
   */

  private final PivotToHighPosition pivotToHigh = new PivotToHighPosition(pivoterSub, PivoterConstants.kMaxPivoterValue);
  private final PivotToHighPosition pivotToMid = new PivotToHighPosition(pivoterSub, 15);
  private final PivotToHighPosition pivotToLow = new PivotToHighPosition(pivoterSub, 6.5);

  private final ExtenderSetPositionCommand extendToHigh = new ExtenderSetPositionCommand(extenderSub, 107);
  private final ExtenderSetPositionCommand extendToMid = new ExtenderSetPositionCommand(extenderSub, 29);
  private final ExtenderSetPositionCommand extendToLow = new ExtenderSetPositionCommand(extenderSub, 60);

  // // Feedforward testing. 
  
  // private final AddFeedFoward extendHighFeed = new AddFeedFoward(pivoterSub);
  // private final AddFeedFoward extendMidFeed = new AddFeedFoward(pivoterSub);
  // private final AddFeedFoward extendLowFeed = new AddFeedFoward(pivoterSub);

  // private final ParallelRaceGroup ExtenderSetPositionHighCommandWithFeedFoward = new ParallelRaceGroup(extendToHigh, extendHighFeed);
  // private final ParallelRaceGroup ExtenderSetPositionMidCommandWithFeedFoward = new ParallelRaceGroup(extendToMid, extendMidFeed);
  // private final ParallelRaceGroup ExtenderSetPositionLowCommandWithFeedFoward = new ParallelRaceGroup(extendToLow, extendLowFeed);


  // public final Command goToHigh = new SequentialCommandGroup(pivotToHigh, ExtenderSetPositionHighCommandWithFeedFoward);
  // public final Command goToMid = new SequentialCommandGroup(pivotToMid, ExtenderSetPositionMidCommandWithFeedFoward);
  // public final Command goToLow = new SequentialCommandGroup(pivotToLow, ExtenderSetPositionLowCommandWithFeedFoward);  

  // Merge commands using sequential commands. 
  public final Command goToHigh = new SequentialCommandGroup(pivotToHigh, extendToHigh);
  public final Command goToMid = new SequentialCommandGroup(pivotToMid, extendToMid);
  public final Command goToLow = new SequentialCommandGroup(pivotToLow, extendToLow);


  /*
   * ---- Trash bin -----
   */
  // Switch based translations
  // private final Command extendToMin = extenderSpeedIn.until(extenderSub::getMinSwitch); // Keep retracting till we hit the switch, then command ends. 
 
  // Extend till we hit the maximum. 
  // private final FunctionalCommand extendToMax = new FunctionalCommand(
  //   // what to do in initialize - basically nothing for us
  //   extenderSub::stopMotor, 
  //   // What to do during the command - run the motor unrestrained. 
  //   () -> extenderSub.translateManual(0.3),
  //   // On finished command
  //   (interrupted) -> {extenderSub.stopMotor();
  //                     System.out.println("Command Ended");},
  //   // isFinished() - get switch value.  
  //   extenderSub::getMaxSwitch,
  //   // Give us the requirements. 
  //   extenderSub
  // );

  // private final FunctionalCommand grabberSetOpen = new FunctionalCommand(
  //   // what to do in initialize - basically nothing for us
  //   grabberSub::stopMotor, 
  //   // What to do during the command - run the motor unrestrained. 
  //   () -> grabberSub.runMotor(-0.125), // Counterclockwise closes. 
  //   // On finished command
  //   (interrupted) -> grabberSub.stopMotor(),
  //   // isFinished() - get switch value.  
  //   () -> (grabberSub.getGrabberDegrees() <= 0),
  //   // Give us the requirements. 
  //   grabberSub
  // );

   
  // private final FunctionalCommand grabberSetClosed = new FunctionalCommand(
  //   // what to do in initialize - basically nothing for us
  //   grabberSub::stopMotor, 
  //   // What to do during the command - run the motor unrestrained. 
  //   () -> grabberSub.runMotor(0.125), // CLockwise closes
  //   // On finished command
  //   (interrupted) -> grabberSub.stopMotor(),
  //   // isFinished() - get switch value.  
  //   () -> (grabberSub.getGrabberDegrees() >= 120),
  //   // Give us the requirements. 
  //   grabberSub
  // );


  // Pivoter Manual Speed Commands
  // private final Command pivotSpeedDown = pivoterSub.pivotManual(-0.125);
  // private final Command pivotSpeedUp = pivoterSub.pivotManual(0.25); // WHY is it inversed

  // Pivot till we hit the minimum limit switch. 
  // private final FunctionalCommand pivotToMin = new FunctionalCommand(
  //   // what to do in initialize - basically nothing for us
  //   pivoterSub::stopMotor, // Fix
  //   // What to do during the command - run the motor unrestrained. 
  //   () -> pivoterSub.pivot(0.125), // Why is the motor inversed? It should be negative
  //   // When we finish the command. 
  //   (interrupted) -> pivoterSub.stopMotor(),
  //   // isFinished() - get switch value; when we hit the switch  
  //   pivoterSub::getSwitch,
  //   // Give us the requirements. 
  //   pivoterSub
  // );

  // retract till we hit the miminimum. 
  // private final FunctionalCommand retractToMin = new FunctionalCommand(
  //   // what to do in initialize - basically nothing for us
  //   extenderSub::stopMotor, // Stop the motor first
  //   // What to do during the command - run the motor unrestrained and retract the extender. . 
  //   () -> extenderSub.translateExtender(-0.3),
  //   // On finished command
  //   (interrupted) -> {extenderSub.stopMotor();
  //     System.out.println("Command Ended");},
  //   // isFinished() - get switch value.  
  //   extenderSub::getMinSwitch, // The minimum switch determines when we've ended. 
  //   // Give us the requirements. 
  //   extenderSub
  // );


  // Extender Manual Speed Commands
  // private final Command extenderSpeedOut = extenderSub.translateManual(0.3);
  // private final Command extenderSpeedIn = extenderSub.translateManual(-0.3);


  // Retract extender to min, then pivot -> represents going to the stored position. 
  // private final SequentialCommandGroup goToStorePosition = new SequentialCommandGroup(retractToMin, pivotToMin);

  
  // private final PivoterPIDCommand PIDPivotForward = new PivoterPIDCommand(pivoterSub, 45); //It's about that - please test
  // private final PivoterPIDCommand PIDPivotBack = new PivoterPIDCommand(pivoterSub, 0); // Dimension is wrong!!! 
  

  // private final ExtenderPIDCommand PIDExtenderExtend = new ExtenderPIDCommand(extenderSub, 22); // We want to get it to 22 inches. 
  // private final ExtenderPIDCommand PIDExtenderRetract = new ExtenderPIDCommand(extenderSub, 0); // We want to get it to 22 inches. 
 
  // private final GrabberSetCommand openGrabber = new GrabberSetCommand(grabberSub, true);
  // private final GrabberSetCommand closeGrabber = new GrabberSetCommand(grabberSub, false);

    
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
    CommandScheduler.getInstance().setDefaultCommand(pivoterSub, addFeedFoward); // Continously add feedforward to the pivoter other than running it's commands with the pivoter or when it reaches the limit switches. 
    CommandScheduler.getInstance().setDefaultCommand(driveSub, arcadeDriveCmd); // Set the default command to have the robot always drive
  
    OI.povButtonUp.whileTrue(PivoterRotateUp);
    OI.povButtonDown.whileTrue(pivotMin);  
    OI.povButtonLeft.whileTrue(retractExtender);
    OI.povButtonRight.whileTrue(extendExtender);
    
    OI.button2Aux.onTrue(stowArm);

    OI.button3Aux.whileTrue(openGrabber); // Open grabber 
    OI.button4Aux.toggleOnTrue(grabCube); // Grab at a weak grip. 
    OI.button6Aux.toggleOnTrue(grabCone); // Grab at a strong grip. 

    
    OI.button7Aux.onTrue(goToHigh);
    OI.button8Aux.onTrue(goToMid);
    OI.button9Aux.onTrue(goToLow);
    
    OI.button12Aux.onTrue(drivetrainEncoderReset);
    
    /*
     * Command junk
     */


    // Grabber Commands

    // OI.povButtonRight.onFalse(extenderSub.commandStop());
    // OI.povButtonLeft.onFalse(extenderSub.commandStop());
    // OI.povButtonLeft.whileFalse(extenderSub::stopMotor);
    
    // CommandScheduler.getInstance().setDefaultCommand(extenderSub, new InstantCommand(extenderSub, ));
    // Extender Executables
    // OI.button10Aux.onTrue(new InstantCommand(grabberSub::resetGrabberDistance));
  
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

    // OI.button7Aux.toggleOnTrue(PIDExtenderExtend);
    // OI.button7Aux.toggleOnFalse(PIDExtenderRetract);
    

    // OI.buttonX.whileTrue(limeCenterAndRange);
    // OI.buttonA.whileTrue(aprilCenterAndRange);
    // runs StowArm command once when robot is initialized
    // CommandScheduler.getInstance().schedule(stowArm);
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
