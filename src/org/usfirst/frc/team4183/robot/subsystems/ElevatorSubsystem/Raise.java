package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Raise extends Command{

	public Raise()
	{
		requires(Robot.elevatorSubsystem);
		
	}
	
	public void execute()
	{
		Robot.elevatorSubsystem.addToPosition(Robot.oi.leftRampAxis.get());
	}

	@Override
	protected boolean isFinished() {
		if(Robot.oi.btnIdle.get()) {
    		return CommandUtils.stateChange(this, new Idle());

		}
		
	return false;
}
}
