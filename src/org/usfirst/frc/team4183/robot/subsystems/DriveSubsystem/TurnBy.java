package org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnBy extends Command {

	private double timeout_sec;
	private double angle_deg;
	
    public TurnBy(double degrees, double aTimeout_sec) 
    {
    	requires(Robot.driveSubsystem);
    	
    	angle_deg = degrees;
    	
    	timeout_sec = aTimeout_sec;
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.driveSubsystem.resetMotion();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    	// Keep enforcing the current position request until we get there
    	Robot.driveSubsystem.turn_degrees(angle_deg);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	boolean timeout = (timeSinceInitialized() > timeout_sec);
    	
    	if (timeout || Robot.driveSubsystem.isTurnComplete()) 
    	{
    		return CommandUtils.stateChange(this, new Idle());
    		
    	}
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
