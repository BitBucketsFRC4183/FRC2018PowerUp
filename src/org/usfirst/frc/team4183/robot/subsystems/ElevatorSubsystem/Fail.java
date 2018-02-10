package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class Fail extends Command {

	public Fail()
	{
		//Set Lights Red
		requires(Robot.elevatorSubsystem);
		Robot.elevatorSubsystem.disable();
	}
	
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

}
