package org.usfirst.frc.team4183.robot.subsystems.RampSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

public class Deployed extends Command {

	public Deployed()
	{
		requires(Robot.rampSubsystem);
	}
	
	protected void initalize()
	{
		Robot.rampSubsystem.disabled();
	}
	
	protected void execute()
	{
	
	}
	
	@Override
	protected boolean isFinished() {
		if (Robot.oi.leftRampAxis.get() != 0 || Robot.oi.rightRampAxis.get() != 0)
		{
			return CommandUtils.stateChange(this, new Lift());
		}
		
		if (Robot.oi.btnIdle.get())
		{
			return CommandUtils.stateChange(this, new Deployed());
		}
		return false;
	}

	protected void end()
	{
		
	}
	
	protected void interrupted()
	{
		
	}
	
}
