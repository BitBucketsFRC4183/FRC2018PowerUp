package org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class MoveBy extends Command {

	private double timeout_sec;
	private double distance_inches;
	
    public MoveBy(double inches, double aTimeout_sec) 
    {
    	requires(Robot.driveSubsystem);
    	
    	distance_inches = inches;
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
    	Robot.driveSubsystem.move_inches(distance_inches);
    	System.out.println("DISTANCE REMAINING"+ distance_inches);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	boolean timeout = (timeSinceInitialized() > timeout_sec);
    	System.out.printf("Timeout is %s\n", timeout?"TRUE":"false");
    	
    	if (timeout || Robot.driveSubsystem.isMoveComplete(distance_inches)) 
    	{
    		
    		return CommandUtils.autoStateChange(this, new Idle());
    	}
    	
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	
    	end();
    }
}
