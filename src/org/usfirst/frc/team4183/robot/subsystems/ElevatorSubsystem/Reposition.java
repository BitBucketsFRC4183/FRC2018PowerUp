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
	
	public void execute()
	{
		Robot.elevatorSubsystem.setSystemPower(Robot.oi.leftRampAxis.get());
		
		//Robot.elevatorSubsystem.addToPosition(Robot.oi.leftRampAxis.get());
		if  (!Robot.elevatorSubsystem.posGreaterThanMin())
		{
			Robot.oi.sbtnOpenMandible.push();
		}
		else if (Robot.elevatorSubsystem.posCloseToInit())
		{
			Robot.oi.sbtnCloseMandible.push();
		}
	}

	@Override
	protected boolean isFinished() {
		if(Robot.oi.btnIdle.get() || Math.abs(Robot.oi.leftRampAxis.get()) < .06) {
    		Robot.elevatorSubsystem.holdPos();
			return CommandUtils.stateChange(this, new Idle());

		}
		
	return false;
}
}
