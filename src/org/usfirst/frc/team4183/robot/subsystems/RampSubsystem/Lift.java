package org.usfirst.frc.team4183.robot.subsystems.RampSubsystem;
import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

public class Lift extends Command{

	public Lift()
	{
		requires(Robot.rampSubsystem);
	}
	
	public void initialize()
	{
		System.out.println(this.getClass().getSimpleName());	
	}
	protected void execute()
	{
		Robot.oi.btnDriveLock.push();
		Robot.rampSubsystem.setRampSpeed(Robot.oi.leftRampAxis.get(), Robot.oi.rightRampAxis.get());
	}
	
	@Override
	protected boolean isFinished() {
		if (Robot.oi.leftRampAxis.get() == 0 || Robot.oi.rightRampAxis.get() == 0)
		{
			return CommandUtils.stateChange(this, new Deployed());
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
