package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class Idle extends Command{

	public Idle()
	{
		//requires(Robot.elevatorSubsystem);
		setRunWhenDisabled(true);
	}
	
	protected void initialize()
	{
		//Robot.elevatorSubsystem.disable();
	}
	
	public void execute()
	{
		
	}
	
	protected boolean isFinished()
	{  /* 
		if (Robot.elevatorSubsystem.isPresent())
		{
			
		}*/
		return true;
	}

	protected void end()
	{
		
	}
	
	protected void interrupted()
	{
		end();
	}
}
