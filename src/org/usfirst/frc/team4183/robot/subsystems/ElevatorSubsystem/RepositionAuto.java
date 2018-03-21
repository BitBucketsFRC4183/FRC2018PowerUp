package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem.ElevatorPresets;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RepositionAuto extends Command{
	// Seconds to wait for pneumatics to open
	private final double TIME_FOR_PNEUMATICS = 0.5;
	
	private int requestedPosition = -1; // Use -1 as indicator for joystice
	
	double pathCompPercent;
	
	public RepositionAuto(int targetPosition, double apathCompPercent)
	{
		requires(Robot.elevatorSubsystem);
		requestedPosition = targetPosition;
		pathCompPercent = apathCompPercent;
	}
	
	public void init()
	{
		System.out.println(this.getClass().getSimpleName());
		Robot.elevatorSubsystem.releasePos();
		
	}
	
	public void execute()
	{
		if (Robot.autonomousSubsystem.getPercentComplete() >  pathCompPercent)
		{
		Robot.elevatorSubsystem.holdPosition(requestedPosition);
		}
		}

	@Override
	protected boolean isFinished() {
		
		//Basically checks to see if the there is not any joystick movement or any buttons pressed for the elevPositions
		// Or just got past the top limit
		 
		double currPos = Robot.elevatorSubsystem.getElevatorNativeUnits();
		
		if (Robot.elevatorSubsystem.isMoveComplete(requestedPosition))	
		{
			return CommandUtils.stateChange(this, new Idle());
		}
	return false;
}
}
