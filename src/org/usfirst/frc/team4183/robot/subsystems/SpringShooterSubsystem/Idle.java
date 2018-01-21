package org.usfirst.frc.team4183.robot.subsystems.SpringShooterSubsystem;

import org.usfirst.frc.team4183.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Idle extends Command 
{

    public Idle() {
        // Use requires() here to declare subsystem dependencies
    		requires(Robot.springShooterSubsystem);
    		setRunWhenDisabled(true);  // Idle state needs this!
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		Robot.springShooterSubsystem.disable();
    		//Set gearbox to high torque (pneumatics)
    		//Release brake
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		//Check for cube on platform
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    		//Returns true when cube is detected
    		return false;
    }

    // Called once after isFinished returns true
    	protected void end() {
    		//Goes to LOADED
    }

    	// Called when another command which requires one or more of the same
    	// subsystems is scheduled to run
    	protected void interrupted() {
    		end();
    }
}
