package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem.AlignLock;
import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem.DriveProfile;
import org.usfirst.frc.team4183.utils.CommandUtils;
import org.usfirst.frc.team4183.utils.RobotTrajectory;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Idle is where the decisions are dispatched for our autonomous states
 * When auto begins this state will determine what (if anything) needs to be done next
 * by evaluating the next plan step. The AutonomousSubsystem is being run periodically
 * to determine if any replaning is needed, otherwise the plan just steps as initially
 * defined. If AutonomousSubsystem declares a replan it will be queried by each state
 * as part of the transition logic; ALL replan, timeouts, or state completions should
 * come back to this idle state to determine what is next.
 */
public class Idle extends Command {

	static int counter = 0;
    public Idle() 
    {
    	requires( Robot.autonomousSubsystem);
    	setRunWhenDisabled(true);  // Idle state needs this!
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
    	if (Robot.runMode == Robot.RunMode.AUTO)
    	{
    		++counter;
    		if (counter == 1) 
    		{
    			AutoTasks blah = new AutoTasks();
    			blah.start();
    		}
    	}
    	
    	
    	// Auto Idle just keeps running, doing nothing until
    	// something tells us what is next; we dispatch the
    	// state change and expect that it will complete, timeout
    	// or be told of a replanning need and will end up back
    	// in this state (idle); this allows the re-evaluation
    	// of the plan or simply popping the next step.
    	// Re-planning hangs here to keep synchronous with the
    	// planning cycle without any additional "fancy" synchronization
    	// techniques that could block the WPI scheduler which sequences
    	// the subsystem periodic and these commands, making the 
    	return false;
    	
    }

    // Called once after isFinished returns true
    protected void end() {
    	counter = 0;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
