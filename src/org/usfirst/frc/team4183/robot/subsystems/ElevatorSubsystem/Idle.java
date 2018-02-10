package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

public class Idle extends Command{

	public Idle()
	{
		requires(Robot.elevatorSubsystem);
		setRunWhenDisabled(true);
	}
	
	protected void initialize()
	{
		Robot.elevatorSubsystem.disable();
		Robot.elevatorSubsystem.engageBrake();
	}
	
	public void execute()
	{
		
	}
	
	protected boolean isFinished()
	{ 
		if (Robot.elevatorSubsystem.getCubeStatus())
		{
			return CommandUtils.stateChange(this, new Loaded());
		}
		
		if (!Robot.elevatorSubsystem.getCubeStatus())
		{
			return CommandUtils.stateChange(this, new Empty());
		}
		return false;
	}

	protected void end()
	{
		
	}
	
	protected void interrupted()
	{
		end();
	}
	
	public class Empty extends Command
	{

		public Empty()
		{
			//add the light change
			requires(Robot.elevatorSubsystem);
		}
		
		@Override
		protected boolean isFinished() {
			 if (Math.abs(Robot.oi.leftRampAxis.get()) > .05)
				{
					return CommandUtils.stateChange(this, new Reposition());
				}
			return false;
		}
		
	}
	public class Loaded extends Command
	{

		public Loaded()
		{
			//add light change
			requires(Robot.elevatorSubsystem);
		}
		
		@Override
		protected boolean isFinished() {
			 if (Math.abs(Robot.oi.leftRampAxis.get()) > .05)
				{
					return CommandUtils.stateChange(this, new Reposition());
				}
			return false;
		}
		
	}

}

