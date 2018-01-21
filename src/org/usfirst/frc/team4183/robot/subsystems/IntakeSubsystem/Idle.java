package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Idle extends Command 
{
//NOTE: Idle goes to Deployed, which functionalny does the same thing as idle EXCEPT
//opens the gate if that gets implemented. As of me writing this the team doesn't know if that
//will be the case so I'm just implanting the deployed state now because it's	harder to jam it in
//later.	
    
	public Idle() 
    {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.intakeSubsystem);
    	setRunWhenDisabled(true);  // Idle state needs this!
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.intakeSubsystem.disable();
    //	Robot.intakeSubsystem.opengate();

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	/// TODO: Resurrect this from last year
    	if( Robot.runMode == Robot.RunMode.TELEOP ) 
    		return CommandUtils.stateChange(this, new Deployed());
    	
    	
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
