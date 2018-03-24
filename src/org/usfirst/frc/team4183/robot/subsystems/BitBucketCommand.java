package org.usfirst.frc.team4183.robot.subsystems;

import edu.wpi.first.wpilibj.command.Command;

public abstract class BitBucketCommand extends Command {

	public BitBucketCommand()
	{
		
	}
	
	@Override
	protected void initialize()
	{
		
	}
	
	@Override
	public void execute()
	{
		
	}
	
	@Override
	abstract public boolean isFinished();
	
	public void end()
	{
		
	}
	
	public void interrupted()
	{
		end();
	}
	
}
