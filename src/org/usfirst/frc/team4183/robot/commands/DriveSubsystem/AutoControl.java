
package org.usfirst.frc.team4183.robot.commands.DriveSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 * In Autonomous, we yield control of the DriveSubsystem
 * to the Autonomous scripter.
 * So this state does very little - mostly want to stay out of the way.
 */
public class AutoControl extends Command 
{	
    public AutoControl() 
    {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() 
    {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() 
    {
    	end();
    }
}
