package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem.AutonomousSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem.ElevatorPresets;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Reposition extends Command{
	
	private double initTime;
	
	// Seconds to wait for pneumatics to open
	private final double TIME_FOR_PNEUMATICS = 0.5;
	
	private int requestedPosition = -1; // Use -1 as indicator for joystice
	
	private double targetPathPerc = -1;
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
	public Reposition(int targetPosition, double targetPathPercentComp)
	{
		requires(Robot.elevatorSubsystem);
		requestedPosition = targetPosition;
		targetPathPerc = targetPathPercentComp;
	}
	
	public void init()
	{
		System.out.println(this.getClass().getSimpleName());
		Robot.elevatorSubsystem.releasePos();
		initTime = timeSinceInitialized();
		
	}
	
	public void execute()
	{
		/// TODO: Need to clean this up... the pneuma delay is only needed if we have them
		/// AND we are moving down AND only if we are in the danger zone (which needs to
		/// account for a cube present on the way down)
		if (targetPathPerc < 0)
		{
		if(timeSinceInitialized() - initTime > TIME_FOR_PNEUMATICS) 
		{
			Robot.oi.sbtnOpenMandible.release();

			/// TODO: Move this to Elevator Subsystem API and query that here
			/// The consideration of the logic for this may need to be moved into 
			/// the state that transitioned here (vs being here)... i.e., if
			/// the elevator is moving down we will want to move the intake mechanism
			/// down just in case there is a cube present. The delay (above) then only
			/// needs to be applied under those conditions
			double currPos = Robot.elevatorSubsystem.getElevatorNativeUnits();
			boolean dangerZone = (currPos < RobotMap.ELEVATOR_SAFE_ZONE);
			
			double cmd = Robot.oi.rightRampAxis.get();
			
			boolean restrictCmd = (dangerZone && (cmd < 0));
			
			// Use the joystick unless otherwise told to reach a position
			if (requestedPosition == -1)
			{
				// Using the joystick is tricky because there is so much gain in the system
				// When under manual control we severely restrict the commands to prevent
				// breaking the elevator.
				// In the past we allowed full control outside critical zones but now with
				// the higher gain at the gearbox the operator is not fast enough not has
				// good enough control over the joystick to really move the elevator with
				// any reasonable accuracy except if going slow.
				// Since manual control is for minor adjustments we will restrict the movement
				// to about 4 to 6 inches per second, which should allow the operator time to visually
				// see and then respond to a the condition and successfully stop without overshoot.

				Robot.elevatorSubsystem.setSystemPower(RobotMap.ELEVATOR_MAX_USER_SPEED_PERCENT_POWER * cmd);
			}
			else
			{
				Robot.elevatorSubsystem.holdPosition(requestedPosition);
			}
		}
		}
		else if (targetPathPerc > 0)
		{
			if (Robot.autonomousSubsystem.getPercentComplete(.6) == AutonomousSubsystem.TrajectoryPercent.PASSED)
			{
			Robot.elevatorSubsystem.holdPosition(requestedPosition);
			}
			else if (Robot.autonomousSubsystem.getPercentComplete(.6) == AutonomousSubsystem.TrajectoryPercent.FAULT)
			{
				System.out.println("ERROR GETTING TRAJECTORY");
				Robot.elevatorSubsystem.disable();
			}
			}
	}

	@Override
	protected boolean isFinished() {
		
		//Basically checks to see if the there is not any joystick movement or any buttons pressed for the elevPositions
		// Or just got past the top limit
		 
		double currPos = Robot.elevatorSubsystem.getElevatorNativeUnits();
		
		if (((requestedPosition == -1) && 
			 (Math.abs(Robot.oi.rightRampAxis.get()) < .06)) || 
			Robot.oi.btnIdle.get() || 
			((Robot.elevatorSubsystem.getElevatorCurrent() > RobotMap.ELEVATOR_MAX_DOWN_CURRENT) && 
			 (Robot.oi.rightRampAxis.get() < 0)) ||
			((requestedPosition != -1) && 
			 Robot.elevatorSubsystem.isMoveComplete(requestedPosition)) 
			)
		{
			return CommandUtils.stateChange(this, new Idle());
		}
	return false;
}
}
