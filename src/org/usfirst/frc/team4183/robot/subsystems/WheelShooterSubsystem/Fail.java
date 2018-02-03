package org.usfirst.frc.team4183.robot.subsystems.WheelShooterSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;
import org.usfirst.frc.team4183.robot.OI;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Fail extends Command 
{

    public Fail() 
    {
    	requires(Robot.wheelShooterSubsystem);
    	//add the Lights change to blinking red when the system has failed.
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.wheelShooterSubsystem.disable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
      //System.out.println("Im Waiting");

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
		if (timeSinceInitialized() >1)
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
