package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ClosedOut extends Command {

	private double timeout = 0.0;
	
    public ClosedOut() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intakeSubsystem);
    	timeout  = 0.0;
    }

    public ClosedOut(double aTimeout) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intakeSubsystem);
    	
    	timeout = aTimeout;
    }
    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intakeSubsystem.closeMandible();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.intakeSubsystem.setIntakeMotorToSpeed(RobotMap.INTAKE_MOTOR_PERCENT, RobotMap.THROAT_MOTOR_PERCENT);
    	System.out.println("Timeout: " + timeout + " TIME SINCE INIT = " + timeSinceInitialized());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
    	System.out.println("Blah");
		System.out.flush();
    	
    	if (timeout == 0.0)
    	{
    		System.out.println("First End Condition");
    		System.out.flush();
	    	if(Robot.oi.btnIdle.get() || (! Robot.oi.btnOutIntake.get() && !Robot.oi.sbtnOuttakeThroat.get())) 
	    		return CommandUtils.stateChange(this, new Idle());
	    	else if(Robot.oi.btnOpenGate.get())
	    		return CommandUtils.stateChange(this, new OpenOut());
    	}
    	else
    	{
    		System.out.println("second if statement, time: " + timeSinceInitialized());
    		System.out.flush();
    		if (timeSinceInitialized() >= timeout)
    		{
    			System.out.println("Second end condition");
    			System.out.flush();
    			return CommandUtils.stateChange(this, new Idle());
    		}
    	}
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("Closed out end");
    	System.out.flush();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("Closed out interrupted");
    	System.out.flush();
    	end();
    }
}
