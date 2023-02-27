// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;

public class DriveRotation extends CommandBase {
  
private final DrivetrainSubsystem driveSub;
private final GyroSubsystem gyroSub;

public DriveRotation(double degrees, DrivetrainSubsystem drive, GyroSubsystem gyro) {
    driveSub = drive;
    gyroSub = gyro;
    driveSub.getRotationController().setSetpoint(degrees);
  
    addRequirements(drive);
    addRequirements(gyro);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyroSub.resetGyro(); // This may pose problems in the future - but for now, we just wa
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSub.arcadeDrive(0, driveSub.getRotationController().calculate(MathUtil.clamp(gyroSub.getDegrees(),-0.75,0.75)));
  }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
     
   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
    return driveSub.getRotationController().atSetpoint();
   }

}
