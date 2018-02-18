package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import org.usfirst.frc.team4183.robot.Robot;

import org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem.AutonomousSubsystem.AutoTaskDescriptor;
import org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem.AutonomousSubsystem.AutoTasks;

import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;



public class MoveBy extends Command 
{
	private double timeout_sec;
	
	public MoveBy( double distanceInch, double aTimeout_sec) 
	{		
		requires( Robot.autonomousSubsystem);
		
		AutonomousSubsystem.setDriveTask(AutoTasks.MOVE_BY, distanceInch);
		timeout_sec = aTimeout_sec;
	}

	@Override
	protected void initialize() 
	{
		// Nothing else to initialize
	}

	@Override
	protected void execute() 
	{
		// Nothing else to execute
	}
	
	
	@Override
	protected boolean isFinished() 
	{
		boolean replan = AutonomousSubsystem.isPlanning();
		
		// Wait for standby (i.e., done), timeout, or new plan
		AutoTaskDescriptor currentDriveTask = AutonomousSubsystem.getDriveTask();
		boolean timeout = (timeSinceInitialized() > timeout_sec);
		
		// If timeout, then force subsystem to stop task, if standby the it already stopped
		AutonomousSubsystem.setDriveTaskComplete(timeout || replan);
		
		if (AutonomousSubsystem.isPlanning() || 
		    timeout || 
		    (currentDriveTask.task == AutoTasks.STANDBY))
		{
			// Go back to idle to decide what is next, if anything
			return CommandUtils.stateChange(this, new Idle());
		}
			
		return false;
	}
	
	@Override
	protected void end() 
	{
	}
	

}
