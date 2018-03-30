package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem.AutonomousSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem.DriveSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem.ElevatorPresets;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Reposition extends Command{
	
	private double initTime;
	
	// Seconds to wait for pneumatics to open
	
	private int requestedPosition = -1; // Use -1 as indicator for joystick
	private double targetPathPerc = -1; // Use -1 as indicator for normal control (vs path sync)
	private int targetPos = 0;
	
	public Reposition()
	{
		requires(Robot.elevatorSubsystem);
		targetPathPerc = -1;		// Don't sync to drive profile path
		requestedPosition = -1;		// Manual control
	}
	public Reposition(int targetPosition)
	{
		requires(Robot.elevatorSubsystem);
		targetPathPerc = -1;		// Don't sync to drive profile path
		requestedPosition = targetPosition;	// Go to this position
		targetPos = targetPosition;
	}
	public Reposition(int targetPosition, double targetPathPercentComp)
	{
		requires(Robot.elevatorSubsystem);
		targetPathPerc = targetPathPercentComp;	// Sync to drive profile path
		requestedPosition = targetPosition;		// Go to this position
		targetPos = targetPosition;
	}
	
	public void initialize()
	{
		Robot.elevatorSubsystem.setCurrentTicks(targetPos);	// Remember where we were told to go
		System.out.println(this.getClass().getName() + " Start" + " " + System.currentTimeMillis()/1000);
	}
	
	public void execute()
	{
		/// TODO: Need to clean this up... the pneuma delay is only needed if we have them
		/// AND we are moving down AND only if we are in the danger zone (which needs to
		/// account for a cube present on the way down)
		if (targetPathPerc < 0)
		{
			/// TODO: Move this to Elevator Subsystem API and query that here
			/// The consideration of the logic for this may need to be moved into 
			/// the state that transitioned here (vs being here)... i.e., if
			/// the elevator is moving down we will want to move the intake mechanism
			/// down just in case there is a cube present. The delay (above) then only
			/// needs to be applied under those conditions
			double currPos = Robot.elevatorSubsystem.getElevatorNativeUnits();
			boolean dangerZone = (currPos < RobotMap.ELEVATOR_DANGER_ZONE);
			
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

				if (cmd > 0)
				{
					Robot.elevatorSubsystem.setSystemPower(0.9 * cmd);
				}
				else if (cmd <0)
				{
					Robot.elevatorSubsystem.setSystemPower(0.07*cmd);
				}
			}
			else
			{
				Robot.elevatorSubsystem.holdPosition(requestedPosition);
			}
		}
		else if (targetPathPerc > 0)
		{
			if (Robot.driveSubsystem.getPercentComplete(targetPathPerc) == DriveSubsystem.TrajectoryPercent.PASSED)
			{
				Robot.elevatorSubsystem.holdPosition(requestedPosition);
			}
			else if (Robot.driveSubsystem.getPercentComplete(targetPathPerc) == DriveSubsystem.TrajectoryPercent.FAULT)
			{
				System.out.println("ERROR GETTING TRAJECTORY PERCENT COMPLETE");
			}
		}
	}

	@Override
	protected boolean isFinished() 
	{
		
		//Basically checks to see if the there is not any joystick movement or any buttons pressed for the elevPositions
		// Or just got past the top limit
		 
		double currPos = Robot.elevatorSubsystem.getElevatorNativeUnits();
		
		if (((requestedPosition == -1) && 	// Manual and inside deadband on joystick
			 (Math.abs(Robot.oi.rightRampAxis.get()) < .06)) || 
			Robot.oi.btnIdle.get() || 
			((Robot.elevatorSubsystem.getElevatorCurrent() > RobotMap.ELEVATOR_MAX_DOWN_CURRENT) && // Current too high and going down manually
			 (Robot.oi.rightRampAxis.get() < 0)) ||
			((requestedPosition != -1) && 					// Specific position and done
			 Robot.elevatorSubsystem.isMoveComplete(requestedPosition)) 
			)
		{
			// Just continue to hold where we are about now
			// Because it may take some time to reach idle we will force the position right
			// now.
			Robot.elevatorSubsystem.holdPosition((int) Robot.elevatorSubsystem.getElevatorNativeUnits());
			return CommandUtils.autoStateChange(this, new Idle());
		}
	return false;
}
	protected void end()
	{
		System.out.println(this.getClass().getName() + " END" + " " + System.currentTimeMillis()/1000);
	}
	protected void interrupted()
	{
		end();
	}
	
	
}
