package org.usfirst.frc.team4183.robot.commands.ElevatorSubSystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.commands.WheelShooterSubsystem.Idle;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Raise extends Command{

	public Raise()
	{
		requires(Robot.elevatorSubSystem);
		
	}
	
	public void execute()
	{
		Robot.elevatorSubSystem.goToPosition((int)SmartDashboard.getNumber("Elevator Pos", 0));
	}

	@Override
	protected boolean isFinished() {
		if(Robot.oi.btnIdle.get()) {
    		return CommandUtils.stateChange(this, new Idle());

		}
		
	return false;
}
}
