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
	private double percent_velocity;
	
    public MoveBy(double inches, double aTimeout_sec) 
    {
    	requires(Robot.driveSubsystem);
    	
    	distance_inches = inches;
    	timeout_sec = aTimeout_sec;
    	percent_velocity = 1.0;
    }
    
    public MoveBy(double inches, double aTimeout_sec, double percent_speed) 
    {
    	requires(Robot.driveSubsystem);
    	
    	distance_inches = inches;
    	timeout_sec = aTimeout_sec;
    	percent_velocity = percent_speed;
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	System.out.println(this.getClass().getName() + " Start" + " " + System.currentTimeMillis()/1000);
    	Robot.driveSubsystem.resetMotion();
		Robot.driveSubsystem.setMotionVelocity(percent_velocity);

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    	System.out.println("Target: " + distance_inches + "\tCurrent: " + Robot.driveSubsystem.getLeftNativeUnits() + " \t" + Robot.driveSubsystem.getRightNativeUnits());
    	// Keep enforcing the current position request until we get there
    	Robot.driveSubsystem.move_inches(distance_inches);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	boolean timeout = (timeSinceInitialized() > timeout_sec);
    	//System.out.printf("Timeout is %s\n", timeout?"TRUE":"false");
    	
    	if (timeout || Robot.driveSubsystem.isMoveComplete(distance_inches)) 
    	{
    		Robot.driveSubsystem.setMotionVelocity(1.0);
    		return CommandUtils.autoStateChange(this, new Idle());
    	}
    	
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("MoveBy end");
    	System.out.println(this.getClass().getName() + " END" + " " + System.currentTimeMillis()/1000);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("MoveBy interrupted");
    	
    	end();
    }
}
