package org.usfirst.frc.team4183.robot.subsystems.RampSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

public class Idle extends Command{

	public Idle()
	{
		requires(Robot.rampSubsystem);
	}
	
	protected void initalize()
	{
		Robot.rampSubsystem.disabled();
		Robot.rampSubsystem.closeRamp();
	}
	
	protected void execute()
	{
		
	}
	
	
	@Override
	protected boolean isFinished() {
		System.out.println(this.getClass().getSimpleName());
		if (Robot.runMode == Robot.runMode.TELEOP)
		{
		if (Robot.oi.releaseRampDriv.get() && Robot.oi.releaseRampOper.get())
		{
			return CommandUtils.stateChange(this, new Deployed());
		}
		}
		return false;
	}
	
	protected void end()
	{
		Robot.rampSubsystem.releaseRamp();
	}
	
	protected void interrupted()
	{
		
	}

}
