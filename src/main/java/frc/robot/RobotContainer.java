// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.ExtenderConstants;
import frc.robot.Constants.PivoterConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
// import edu.wpi.first.wpilibj.SerialPort.StopBits;
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
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  // private final BalancePIDSubsystem balanceSub = new BalancePIDSubsystem();
  private final GyroSubsystem gyroSub = new GyroSubsystem();
  // private final BalanceHoldPIDSubsystem balanceHoldSub = new BalanceHoldPIDSubsystem();
  // private final BalancePIDSubsystem balanceSub = new BalancePIDSubsystem(driveSub, gyroSub);
  private final DrivetrainRamp driveRamp = new DrivetrainRamp(1.33, 2.5); // These values may be wrong. 
  private final PhotonVisionSubsystem photonSub = new PhotonVisionSubsystem(); // I just want to read the values in periodic().
  
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
  private final InstantCommand stopGrabber = new InstantCommand(grabberSub::stopMotor, grabberSub); // Stops the grabber. 
  private final OpenGrabber openGrabber = new OpenGrabber(grabberSub);
  private final CloseGrabberCone grabCone = new CloseGrabberCone(grabberSub);
  private final CloseGrabberCube grabCube = new CloseGrabberCube(grabberSub);

  /*
   * Photonvision Commands
   */
  private final TargetCenterPIDCommand photonCenter = new TargetCenterPIDCommand(photonSub, driveSub);

  /*
   * Store Commands: We need to create the commands again to follow the syntax of creating sequential commands. 
   */ 

  
  private final InstantCommand pivotEncoderReset = new InstantCommand(pivoterSub::resetEncoder, pivoterSub);
  private final InstantCommand extenderEncoderReset = new InstantCommand(extenderSub::resetExtenderEncoder, extenderSub);

  private final ExtenderRetractLimit retractStore = new ExtenderRetractLimit(extenderSub);
  private final PivoterPivotMin pivotStore = new PivoterPivotMin(pivoterSub);

  public final Command stowArm = new SequentialCommandGroup(retractStore, pivotStore, pivotEncoderReset, extenderEncoderReset);
  public final Command stowGroundArm =  new SequentialCommandGroup(new PivotToHighPosition(pivoterSub, 3), new ExtenderRetractLimit(extenderSub), new PivoterPivotMin(pivoterSub));
  // public final Command stowArm = new SequentialCommandGroup(new ExtenderRetractLimit, pivotStore, pivotEncoderReset, extenderEncoderReset);

  /*
   * Position based commands:
   * Set the pivoter/extender to certain setpoints. 
   */

  private final PivotToHighPosition pivotToHigh = new PivotToHighPosition(pivoterSub, PivoterConstants.kHighPivoterValue);
  private final PivotToHighPosition pivotToMid = new PivotToHighPosition(pivoterSub, PivoterConstants.kMidPivoterValue);
  private final PivotToHighPosition pivotToLow = new PivotToHighPosition(pivoterSub, PivoterConstants.kLowPivoterValue);
  private final PivotToHighPosition pivotUpToGround = new PivotToHighPosition(pivoterSub, PivoterConstants.kGroundPivoterUpValue);
  private final PivotToHighPosition pivotToFeeder = new PivotToHighPosition(pivoterSub, PivoterConstants.kFeedPivoterValue);
  private final PivotToLowPosition pivotDownToGround = new PivotToLowPosition(pivoterSub, PivoterConstants.kGroundPivoterValue);
  //private final SequentialCommandGroup pivotDown = new SequentialCommandGroup(pivotUpToGround, pivotDownToGround);

  private final ExtenderSetPositionCommand extendToHigh = new ExtenderSetPositionCommand(extenderSub, ExtenderConstants.kHighExtenderValue);
  private final ExtenderSetPositionCommand extendToMid = new ExtenderSetPositionCommand(extenderSub, ExtenderConstants.kMidExtenderValue);
  private final ExtenderSetPositionCommand extendToLow = new ExtenderSetPositionCommand(extenderSub, ExtenderConstants.kLowExtenderValue);
  private final ExtenderSetPositionCommand extendToGround = new ExtenderSetPositionCommand(extenderSub, ExtenderConstants.kGroundExtenderValue); 

  /*
   * Auto Commands
   */
  private final SequentialCommandGroup auto1ExtendAndDrop = new SequentialCommandGroup(new ExtenderSetPositionCommand(extenderSub, ExtenderConstants.kHighExtenderValue), new OpenGrabber(grabberSub), new InstantCommand(grabberSub::stopMotor, grabberSub)); //
  private final SequentialCommandGroup auto2ExtendDropTurnAndDrive = new SequentialCommandGroup(new SequentialCommandGroup(new ExtenderSetPositionCommand(extenderSub, ExtenderConstants.kHighExtenderValue), new OpenGrabber(grabberSub), new InstantCommand(grabberSub::stopMotor, grabberSub)), new AutoDriveRotate(driveSub, gyroSub, 180), new AutoDriveFoward(driveSub, 10));
  
  // private final AddFeedFoward extendHighFeed = new AddFeedFoward(pivoterSub);
  // private final AddFeedFoward extendMidFeed = new AddFeedFoward(pivoterSub);
  // private final AddFeedFoward extendLowFeed = new AddFeedFoward(pivoterSub);

  // private final ParallelRaceGroup ExtenderSetPositionHighCommandWithFeedFoward = new ParallelRaceGroup(extendToHigh, extendHighFeed);
  // private final ParallelRaceGroup ExtenderSetPositionMidCommandWithFeedFoward = new ParallelRaceGroup(extendToMid, extendMidFeed);
  // private final ParallelRaceGroup ExtenderSetPositionLowCommandWithFeedFoward = new ParallelRaceGroup(extendToLow, extendLowFeed);


  // public final Command goToHigh = new SequentialCommandGroup(pivotToHigh, ExtenderSetPositionHighCommandWithFeedFoward);
  // public final Command goToMid = new SequentialCommandGroup(pivotToMid, ExtenderSetPositionMidCommandWithFeedFoward);
  // public final Command goToLow = new SequentialCommandGroup(pivotToLow, ExtenderSetPositionLowCommandWithFeedFoward);  

  // // Merge commands using sequential commands. 
  public final Command goToHigh = new SequentialCommandGroup(pivotToHigh, extendToHigh);
  public final Command goToMid = new SequentialCommandGroup(pivotToMid, extendToMid);
  public final Command goToLow = new SequentialCommandGroup(pivotToLow, extendToLow);
  public final Command goToGround = new SequentialCommandGroup(pivotUpToGround, extendToGround, /*openGrabber,*/ pivotDownToGround);


  /*
   * Auto Sequences
   */
  public final AutoDriveFoward driveOut = new AutoDriveFoward(driveSub, -100); // Cross the 3+ points mark. 
  public final AutoDriveFoward driveToObject = new AutoDriveFoward(driveSub, -180); // Cross the 3+ points mark. 
  public final SequentialCommandGroup dropAndDrive = new SequentialCommandGroup(
    // Stow 
    new ExtenderRetractLimit(extenderSub),
    new PivoterPivotMin(pivoterSub),
    new InstantCommand(pivoterSub::resetEncoder, pivoterSub),
    new InstantCommand(extenderSub::resetExtenderEncoder, extenderSub),
    // Pivot up
    new PivotToHighPosition(pivoterSub, PivoterConstants.kHighPivoterValue),
    new ExtenderSetPositionCommand(extenderSub, ExtenderConstants.kHighExtenderValue),
    new WaitCommand(2),
    // Stow again
    new ExtenderRetractLimit(extenderSub),
    new PivoterPivotMin(pivoterSub),
    new InstantCommand(pivoterSub::resetEncoder, pivoterSub),
    new InstantCommand(extenderSub::resetExtenderEncoder, extenderSub),
    new WaitCommand(2),
    //Drive back
    new AutoDriveBackward(driveSub, 180));
  // public final BalanceDistance balance = new BalanceDistance(driveSub, balanceSub);
  // public final BalanceHold balanceHold = new BalanceHold(balanceHoldSub, driveSub);
   

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

    /*
     * Controller 
     */

    OI.buttonRB.whileTrue(photonCenter);
    
    /*
     * Aux Stick
     */

    OI.povButtonUp.whileTrue(PivoterRotateUp);
    OI.povButtonDown.whileTrue(pivotMin);  
    OI.povButtonLeft.whileTrue(retractExtender);
    OI.povButtonRight.whileTrue(extendExtender);
    
    OI.triggerAux.toggleOnTrue(grabCone);


    OI.button2Aux.toggleOnTrue(openGrabber); // Open grabber 
    // OI.button4Aux.toggleOnTrue(grabCone); // Grab at a strong grip. 
    // OI.button5Aux.onTrue(stopGrabber);
    OI.button3Aux.onTrue(goToGround); // Pivot to the ground position. 
    OI.button4Aux.onTrue(pivotToFeeder);
    OI.button5Aux.onTrue(stowArm);
    OI.button6Aux.onTrue(stowGroundArm);

    OI.button7Aux.onTrue(goToHigh);
    OI.button8Aux.onTrue(goToMid);
    OI.button9Aux.onTrue(goToLow);

    OI.button11Aux.onTrue(new AutoDriveBackward(driveSub, 12));
    // OI.button12Aux.onTrue(); 
    // OI.button12Aux.onTrue(drivetrainEncoderReset);

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


    return dropAndDrive;
  }
}
