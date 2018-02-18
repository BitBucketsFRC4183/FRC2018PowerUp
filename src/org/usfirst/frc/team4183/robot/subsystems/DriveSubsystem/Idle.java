package org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem.PathPlans;
import org.usfirst.frc.team4183.utils.CommandUtils;
import org.usfirst.frc.team4183.utils.RobotTrajectory;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;


/**
 *
 */
public class Idle extends Command 
{

	static double lastTestModeTime_sec = 0.0;
	
	// Toggle sign on each one
	double moveDistance_inches = -2.0 * RobotMap.WHEEL_CIRCUMFERENCE_INCHES;
	double turnAngle_deg = -45.0;
	
    public Idle() 
    {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.driveSubsystem);
    	setRunWhenDisabled(true);  // Idle state needs this!
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.driveSubsystem.disable();

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	if( Robot.runMode == Robot.RunMode.TELEOP) 
    	{
    		return CommandUtils.stateChange(this, new DriverControl());
    	}
    	if( Robot.runMode == Robot.RunMode.AUTO)
    	{
    		return CommandUtils.stateChange(this, new AutoControl());
    	}
    	
    	// Getting into test mode requires 2 conditions to avoid inadvertent activation
    	// of future other test modes
    	if( Robot.runMode == Robot.RunMode.TEST)
    	{
    		// Throttle the test mode to prevent it from triggering more
    		// often than is necessary; serves a couple of purposes.
    		// a) some tests could be destructive if running over and over quickly
    		// b) running some tests back-to-back can make it hard to see what is happening
    		// A changeable default test period of 2 seconds provides a reasonable chance to see
    		// what is happening
    		double now_sec = Timer.getFPGATimestamp();
    		if ((now_sec - lastTestModeTime_sec) > Robot.driveSubsystem.getTestModePeriod_sec())
    		{
    			lastTestModeTime_sec = now_sec;
    			
	    		switch (Robot.driveSubsystem.getTestSubmode())
	    		{
	    		case DIAGNOSTICS:
	    			if (Robot.driveSubsystem.getDiagnosticsFlag())	// Diagnostics can only be run once per reset cycle
	    			{
	    				// Don't run repeatedly because it could be harmful
	    				return CommandUtils.stateChange(this, new Diagnostics());
	    			}
	    		case MOVE_TEST:
	    			// This test can run multiple cycles without trouble
	    			moveDistance_inches *= -1.0;
	    			return CommandUtils.stateChange(this, new MoveBy(moveDistance_inches, 5.0));	// Magic timeout value
	    		case TURN_TEST:
	    			// This test can run multiple cycles without trouble
	    			turnAngle_deg *= -1;
	    			return CommandUtils.stateChange(this, new TurnBy(turnAngle_deg, 5.0));			// Magic timeout value
	    		case PROFILE_TEST:
	    			RobotTrajectory trajectory = PathPlans.getSelectedTrajectory();
	    			if (trajectory != null)
	    			{
	    				if (trajectory.runCount > 0) 	// Only run trajectories up to the limit defined
	    				{								// This prevents running into things due to space (reset and try again)
	    					--trajectory.runCount;
	    					return CommandUtils.stateChange(this, new DriveProfile(trajectory));
	    				}
	    			}
	    			break;
	    		default:
	    			break;
	    		}
    		}
    	}    	
    	// When disabled, we stay in idle; other states should return here
    	// when disable causes the default command to be invoked (if this is the default)
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
