package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Reposition extends Command{
	
	private double initTime;
	
	// Seconds to wait for pneumatics to open
	private final double TIME_FOR_PNEUMATICS = 0.5;
	
	private int requestedPosition = -1; // Use -1 as indicator for joystice

	public Reposition()
	{
		requires(Robot.elevatorSubsystem);
		requestedPosition = -1;
	}
	public Reposition(int targetPosition)
	{
		requires(Robot.elevatorSubsystem);
		requestedPosition = targetPosition;
	}
	
	public void init()
	{
		System.out.println(this.getClass().getSimpleName());
		Robot.elevatorSubsystem.releasePos();
		initTime = timeSinceInitialized();
		
	}
	
	public void execute()
	{
		
		if(timeSinceInitialized() - initTime > TIME_FOR_PNEUMATICS) {
			Robot.oi.sbtnOpenMandible.release();
			double currPos = Robot.elevatorSubsystem.getElevatorNativeUnits();
			
			// Use the joystick unless otherwise told to reach a position
			if (requestedPosition == -1)
			{
				Robot.elevatorSubsystem.setSystemPower((currPos > RobotMap.ELEVATOR_SAFE_ZONE) 
														? Robot.oi.rightRampAxis.get() 
														: RobotMap.signedSquare(Robot.elevatorSubsystem.limitJoystickCommand(Robot.oi.rightRampAxis.get(), 0.8), 3));
			}
			else
			{
				Robot.elevatorSubsystem.holdPosition(requestedPosition);
			}
		}		
	}

	@Override
	protected boolean isFinished() {
		
		//Basically checks to see if the there is not any joystick movement or any buttons pressed for the elevPositions
		 
		/*if (Robot.oi.btnIdle.get() || Math.abs(Robot.oi.leftRampAxis.get()) < .06 || !Robot.oi.btnMedPosElev.get() 
				 || !Robot.oi.btnHighPosElev.get() || !Robot.oi.btnLowPosElev.get() || !Robot.oi.btnTransPosElev.get() 
				 || Robot.elevatorSubsystem.closeToDesiredPos())
			{
				Robot.elevatorSubsystem.holdPos();
				return CommandUtils.stateChange(this, new Idle());
			}
			*/
		if (((requestedPosition == -1) && 
			 (Math.abs(Robot.oi.rightRampAxis.get()) < .06)) || 
			Robot.oi.btnIdle.get() || 
			((Robot.elevatorSubsystem.getElevatorCurrent() > RobotMap.ELEVATOR_MAX_DOWN_CURRENT) && 
			 (Robot.oi.rightRampAxis.get() < 0)) ||
			((requestedPosition != -1) && 
			 Robot.elevatorSubsystem.isMoveComplete(requestedPosition)))
		{
			return CommandUtils.stateChange(this, new Idle());
		}
	return false;
}
}
