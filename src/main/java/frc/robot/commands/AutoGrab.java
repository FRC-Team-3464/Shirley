// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OI;
import frc.robot.Constants.GrabberConstants;
import frc.robot.subsystems.DrivetrainRamp;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class AutoGrab extends CommandBase {
  /** Creates a new AutoGrab. 
  private final XboxController controller = OI.xBoxController;
  private final GrabberSubsystem grabberSub;
  private final LEDSubsystem ledSub;
  private final DrivetrainSubsystem driveSub;
  private final DrivetrainRamp driveRampSub;
  public AutoGrab(GrabberSubsystem grabberSub, LEDSubsystem ledSub, DrivetrainSubsystem driveSub, DrivetrainRamp driveRampSub) {
    this.grabberSub = grabberSub;
    this.ledSub = ledSub;
    this.driveSub = driveSub;
    this.driveRampSub = driveRampSub;
    addRequirements(grabberSub);
    addRequirements(ledSub);
    addRequirements(driveSub);
    addRequirements(driveRampSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(grabberSub.getGrabberLimit()) {
    //     // }
 


    if( grabberSub.getGrabberLimit()){
      grabberSub.runMotor(GrabberConstants.kStrongGrabSpeed);
      ledSub.rainbow();
      driveSub.stopDrive();
    }else{
      ledSub.white();
      driveSub.arcadeDrive(driveRampSub.applyAsDouble((controller.getLeftY() * 0.8) - (0.2 * controller.getRightTriggerAxis())) , controller.getRightX() * 0.6); // Halve the speed - we don't need to go so fast now. 
    }
    // LED 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // grabberSub.stopMotor();
    // grabberSub.runMotor(GrabberConstants.kStrongGrabSpeed);
    // ledSub.red();
    // driveSub.stopDrive();
    // driveSub.arcadeDrive(0.2,0);
    // new WaitCommand(0.5);
    // LED Flash

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

*/