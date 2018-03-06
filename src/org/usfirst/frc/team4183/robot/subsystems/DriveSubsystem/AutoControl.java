
package org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem.AutonomousSubsystem;
import org.usfirst.frc.team4183.utils.CommandUtils;

import com.ctre.phoenix.motion.SetValueMotionProfile;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

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
    	System.out.println(this.getClass().getSimpleName());
    	// Each time we re-enter AutoControl, reset the drive subsystem encoder
    	// states to allow for relative motion (the AutonomousSubsystem is
    	// responsible for knowing the absolute state)
    	Robot.driveSubsystem.resetMotion();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    	// For all cases, determine what we are being asked to do
    	// NOTE: Each of the following Drive commands should return to
    	// AutoControl when the run-mode is auto, otherwise they should
    	// return to Idle when complete.
    	//
    	// Ask the AutonomousSubsystem what we should be doing right now
    	// But hold on to it until cycle is complete
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	// If the auto mode is turned off, go back to Idle to determine
    	// the correct state
    	if (Robot.runMode != Robot.RunMode.AUTO)
       	{
       		return CommandUtils.stateChange(this, new Idle());
       	} 	
   	
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
