package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem.AlignLock;
import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem.DriveProfile;
import org.usfirst.frc.team4183.utils.CommandUtils;
import org.usfirst.frc.team4183.utils.Positions.GenericPositions;
import org.usfirst.frc.team4183.utils.RobotTrajectory;

import edu.wpi.first.wpilibj.Timer;
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
	Timer timeOutGameInfo = new Timer();
    public Idle() 
    {
    	requires( Robot.autonomousSubsystem);
    	setRunWhenDisabled(true);  // Idle state needs this!
    }
    
    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	System.out.println(this.getClass().getName() + " Auto Start" + " " + System.currentTimeMillis()/1000);
    	timeOutGameInfo.reset();
    	timeOutGameInfo.start();
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
    		if (counter == 0) 
    		{
    			// Choose auto mode
    			switch (Robot.autonomousSubsystem.getAutoChoice())
    			{
    			case PLAY_GAME:
    				if (timeOutGameInfo.get() < RobotMap.TIMEOUT_GAME_INFO_SEC && Robot.autonomousSubsystem.getSwitchPosition() == GenericPositions.UNKNOWN)
    				{
    					System.out.println("Waiting for Game Data");
    				}
    				else
    				{
    					AutoGameTasks game= new AutoGameTasks(timeOutGameInfo.get());
    					game.start();
    					++counter;
    				}
    				break;
    			case MOVE_TURN_TEST:
    				AutoMoveTurnTest move_turn = new AutoMoveTurnTest();
    				move_turn.start();
    				++counter;
    				break;
    			case DRIVE_PROFILE_TEST:
    			
    			case OFF:
				default:
					++counter;
					break;
    			}

    		}
    	}
    	
    	
    	// Auto Idle never finished on its own
    	// Just let the FMS disable us when the period is over
    	return false;
    	
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println(this.getClass().getName() + " AUTO END" + " " + System.currentTimeMillis()/1000);
    	counter = 0;
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
