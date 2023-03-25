// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BalanceHoldPIDSubsystem extends PIDSubsystem {
    // Command to hold the robot on the charge station. 
    private final DrivetrainSubsystem driveSub;
    private double speed;
   
    //create PID with predetermined constants
    public BalanceHoldPIDSubsystem(DrivetrainSubsystem driveSub){ // What runs when we first create this command. 
        super(new PIDController(0.18, 0.000, .001));
        getController().setSetpoint(0);
        getController().setTolerance(3);

        this.driveSub = driveSub;
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
    }
    
    @Override  
    public double getMeasurement(){
        return driveSub.getLeftPosition();
    }
    
    @Override
    public void periodic(){}
 
}
