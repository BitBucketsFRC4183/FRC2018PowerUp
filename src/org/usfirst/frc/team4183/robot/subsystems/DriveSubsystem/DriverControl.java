
package org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.LightingControl;
import org.usfirst.frc.team4183.robot.LightingControl.LightingObjects;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriverControl extends Command 
{	
    public DriverControl() 
    {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires( Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.lightingControl.set(LightingObjects.DRIVE_SUBSYSTEM,
    		                      LightingControl.FUNCTION_ON,
    		                      LightingControl.COLOR_ORANGE,
    		                      0,	// nspace - don't care
    		                      0);	// period_msec - don't care
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    	Robot.driveSubsystem.arcadeDrive(Robot.oi.axisForward.get(), Robot.oi.axisTurn.get());

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	if (Robot.oi.btnDriveLock.get() || Robot.oi.sbtnShake.get())
    	{
    		return CommandUtils.stateChange(this, new DriveLock());
    	}
    	else if(Robot.oi.btnAlignLock.get()) 
    	{
    		return CommandUtils.stateChange(this, new AlignLock());
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
