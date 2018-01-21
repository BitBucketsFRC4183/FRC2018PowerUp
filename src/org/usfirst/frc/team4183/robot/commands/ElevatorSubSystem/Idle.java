package org.usfirst.frc.team4183.robot.commands.ElevatorSubSystem;

import org.usfirst.frc.team4183.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class Idle extends Command{

	public Idle()
	{
		requires(Robot.wheelShooterSubsystem);
		setRunWhenDisabled(true);
	}
	
	protected void initialize()
	{
		Robot.elevatorSubSystem.disable();
	}
	
	public void execute()
	{
		
	}
	
	protected boolean isFinished()
	{
		return false;
	}

	protected void end()
	{
		
	}
	
	protected void interrupted()
	{
		end();
	}
}
