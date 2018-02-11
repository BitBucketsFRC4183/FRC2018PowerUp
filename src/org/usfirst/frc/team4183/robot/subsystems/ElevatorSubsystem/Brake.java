package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

public class Brake extends Command{

	public Brake()
	{
		requires(Robot.elevatorSubsystem);
	}
	
	public void init()
	{
		Robot.elevatorSubsystem.engageBrake();
		Robot.elevatorSubsystem.disable();
	}
	
	public void execute()
	{
		if (Robot.oi.sbtnIntakeThroat.get())
		{
			Robot.elevatorSubsystem.intakeThroat();
		}
		else if (Robot.oi.sbtnOuttakeThroat.get())
		{
			Robot.elevatorSubsystem.outtakeThroat();
		}
		else
		{
			Robot.elevatorSubsystem.disableThroat();
		}
	}
	
	@Override
	protected boolean isFinished() {
		if (Math.abs(Robot.oi.leftRampAxis.get()) > .06 || Robot.oi.btnMedPosElev.get() 
				|| Robot.oi.btnHighPosElev.get() || Robot.oi.btnLowPosElev.get() || Robot.oi.btnTransPosElev.get())
		{
			return CommandUtils.stateChange(this, new Idle());
		}
		return false;
	}
	
	public void end()
	{
		Robot.elevatorSubsystem.holdEncodPos(false);
		Robot.elevatorSubsystem.holdEncodPos(true);
		Robot.elevatorSubsystem.disengageBrake();
		
		
	}

}
