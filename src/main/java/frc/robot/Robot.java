// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.ScheduleCommand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  static double pipeIndex;
  NetworkTableEntry pipelineIndex;

  /*
   * Setting the autonomous command
   */

  // Create a chooser that selects an autonomous command.   
  public static final SendableChooser<String> autoChooser = new SendableChooser<>();
  
  // get the name of the auto that's selected.
  public static String autoSelected; 

  // Get the autonomous command picked from sendable chooser
  Command autoSelectedCommand;



  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    
    // Create the choose with it's options. 

    OI oi = new OI();

    
    // *** autoChooser.setDefaultOption("Drive Balance", m_robotContainer.getDriveBalanceAuto()); // This might work - 
    // *** autoChooser.addOption("Drive Balance Auto", m_robotContainer.getDriveBalanceAuto()); // Just get the actual command instead of the string of it. 

    m_robotContainer = new RobotContainer();
    // autoChooser.addOption("Auto 2: Place Cone and Drive", "autoDropAndDrive");
    autoChooser.addOption("Auto 1: Drive Only", "autoDriveForward");
    autoChooser.addOption("Auto 2: Place Cone and Drive", "autoDropAndDrive");
    autoChooser.addOption("Auto 3: Left Two Object", "leftAuto2Object");
    autoChooser.addOption("Auto 4: Right Two Object", "rightAuto2Object");
    autoChooser.addOption("Auto 5: Place Cone and Balance", "autoDropAndBalance");
    autoChooser.setDefaultOption("Auto 3: Left Two Object", "leftAuto2Object");
    // autoChooser.addOption("Auto 2: Place Cone and Drive", "autoDropAndDrive");
    // autoChooser.setDefaultOption("Auto 3: Place Cone and Balance", "autoDropAndBalance");

    // Put the auto choose 
    SmartDashboard.putData("Select Auto: ", autoChooser);
    
    CameraServer.startAutomaticCapture(0); // Open the camera that's connected to the roborio. 

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // Get the string of the auto we selected.
    
    // Get the auto selected as a string. 
    autoSelected = autoChooser.getSelected();

    System.out.println("Selected Auto: " + autoSelected);
    // Get the corresponding command from robot container. 
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
