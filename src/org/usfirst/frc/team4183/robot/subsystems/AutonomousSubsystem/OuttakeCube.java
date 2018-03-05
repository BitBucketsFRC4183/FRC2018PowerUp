package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class OuttakeCube extends Command {
	

    public OuttakeCube() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);    	
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	System.out.printf("PRESSING BUTTON\n");
    	Robot.autonomousSubsystem.pressOuttakeButton();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double time = timeSinceInitialized();
    	System.out.printf("Execute Time = %f\n", time);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	double time = timeSinceInitialized();
    	System.out.printf("isFinished Time = %f\n", time);
    	if(time > 1.0) 
    	{
        	System.out.printf("RELEASING BUTTON\n");

    		Robot.autonomousSubsystem.releaseOuttakeButton();
    		return true;
    	}
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("END");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("INTERRUPTED");
    	end();
    }
}
