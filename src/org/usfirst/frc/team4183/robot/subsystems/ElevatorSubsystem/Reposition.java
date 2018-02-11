package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Reposition extends Command{

	public Reposition()
	{
		requires(Robot.elevatorSubsystem);	
	}
	
	public void init()
	{
		Robot.elevatorSubsystem.releasePos();
	}
	
	public void execute()
	{
		Robot.elevatorSubsystem.setSystemPower(Robot.oi.leftRampAxis.get());
		
		/*
		if (Robot.oi.btnHighPosElev.get())
		{
			Robot.elevatorSubsystem.setElevPos(ElevatorSubsystem.ElevatorPositions.SCALE);
		}
		else if (Robot.oi.btnMedPosElev.get())
		{
			Robot.elevatorSubsystem.setElevPos(ElevatorSubsystem.ElevatorPositions.SWITCH);
		}
		else if (Robot.oi.btnLowPosElev.get())
		{
			Robot.elevatorSubsystem.setElevPos(ElevatorSubsystem.ElevatorPositions.SCALE);
		}
			
		if  (!Robot.elevatorSubsystem.posGreaterThanMin())
		{
			Robot.oi.sbtnOpenMandible.push();
		}
		else if (Robot.elevatorSubsystem.posCloseToInit())
		{
			Robot.oi.sbtnCloseMandible.push();
		}

		if (Math.abs(Robot.oi.leftRampAxis.get()) > .06)
		{
			Robot.elevatorSubsystem.setElevPos(ElevatorSubsystem.ElevatorPositions.MANUAL);
		}
		
		if (Robot.elevatorSubsystem.getElevPos() != ElevatorSubsystem.ElevatorPositions.MANUAL)
		{
			Robot.elevatorSubsystem.goToPosition(Robot.elevatorSubsystem.getElevPos().getUnits());
		}
		else
		{
			Robot.elevatorSubsystem.addToPosition(Robot.oi.leftRampAxis.get());
		}
		*/
	}

	@Override
	protected boolean isFinished() {
		 if (Robot.oi.btnIdle.get() || Math.abs(Robot.oi.leftRampAxis.get()) < .06 || !Robot.oi.btnMedPosElev.get() || !Robot.oi.btnHighPosElev.get() || !Robot.oi.btnLowPosElev.get() || !Robot.oi.btnTransPosElev.get())
			{
				Robot.elevatorSubsystem.holdPos();
				return CommandUtils.stateChange(this, new Idle());
			}
	return false;
}
}
