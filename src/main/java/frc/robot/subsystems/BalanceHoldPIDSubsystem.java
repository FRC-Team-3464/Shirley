// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BalanceHoldPIDSubsystem extends PIDSubsystem {
  private final DrivetrainSubsystem driveSub = new DrivetrainSubsystem();
    // private final EncoderSubsystem encoderSub = new EncoderSubsystem();
    // private final LimelightSubsystem limeSub = new LimelightSubsystem();
    // private final GyroSubsystem gyroSub = new GyroSubsystem();
    private double speed;
    // private double distance; // create distance
    
   
    // private double encoderDistance;
    //create PID with predetermined constants
    public BalanceHoldPIDSubsystem(){ // What runs when we first create this command. 
        super(new PIDController(0.18, 0.000, .001));
        getController().setSetpoint(0);
        getController().setTolerance(3);
        
        // distance = 0;
        // encoderDistance = 0;
    }

    @Override
    public void useOutput(double output, double setpoint){
        speed = getController().calculate(output, setpoint);
        if(speed > .35){
            speed = .35;
        }
        else if(speed < -.35){
            speed = -.35;
        }
        
        driveSub.arcadeDrive(speed, 0);
        //System.out.println(speed);
    }
    
    @Override  
    public double getMeasurement(){
        return driveSub.getLeftPosition();
    }
    
    @Override
    public void periodic(){
        // Distance slipped we gotta find "4". 
        
            //driveSub.arcadeDrive(0, getController().calculate(getMeasurement()));
    }
 
}
