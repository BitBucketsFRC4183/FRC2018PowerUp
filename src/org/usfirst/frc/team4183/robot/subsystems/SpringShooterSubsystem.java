package org.usfirst.frc.team4183.robot.subsystems;

import org.usfirst.frc.team4183.robot.commands.SpringShooterSubsystem.Idle;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SpringShooterSubsystem extends BitBucketsSubsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new Idle());
    }
    
	public void disable() {
		// TODO Auto-generated method stub
		
	}



	@Override
	public void diagnosticsInit() {
		// TODO Auto-generated method stub
		
	}



	@Override
	public void diagnosticsCheck() {
		// TODO Auto-generated method stub
		
	}



	@Override
	public void diagnosticsFlagSet() {
		// TODO Auto-generated method stub
		
	}



	@Override
	public void periodic() {
		// TODO Auto-generated method stub
		
	}
}

