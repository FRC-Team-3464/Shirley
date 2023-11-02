package frc.robot.commands;

import frc.robot.subsystems.BalancePIDSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
// import frc.robot.Subsystems.LimelightPIDDistanceSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
   * Uses distance subsystem to drive robot to desired setpoint, in this case 83.2 inches from the goal
   *
   * @param DriveSubsystem @param LimelightPIDDistanceSubsystem The subsystems used by this command.
   */

public class BalanceDistance extends CommandBase {
  //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem driveSub;
  private final LEDSubsystem ledSub;
  //private final LimelightPIDDistanceSubsystem limePIDSub;
  private final BalancePIDSubsystem balanceSub;

  public BalanceDistance(DrivetrainSubsystem drive, BalancePIDSubsystem lime, LEDSubsystem ledSub) {
    driveSub = drive;
    balanceSub = lime;
    this.ledSub = ledSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSub);
    addRequirements(balanceSub);
    addRequirements(ledSub);
  }

  @Override
  public void initialize() {
        
  }
  @Override
  public void execute() {
    ledSub.blue();
    balanceSub.useOutput(balanceSub.getMeasurement(), 0); // Run the autobalance PID Command
    System.out.println("Running Command PID");
  }

 @Override
  public boolean isFinished() {
      // return balanceSub.returnAtSetpoint();// Add tolerance to PID controller 
      return false;
    }
}