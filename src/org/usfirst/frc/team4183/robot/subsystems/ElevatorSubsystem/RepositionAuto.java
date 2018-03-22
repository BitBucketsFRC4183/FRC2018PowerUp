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
		
		// NOTE: We do not need to do anything special to move
		// the elevator in auto other than command a holdPosition
		// If the elevator is not at that position the motion magic
		// will compute a trapezoidal velocity profile that will get
		// us there.
		//
		// The key difference between this and a normal reposition
		// is that the command actions are delayed within execute
		// until the correct path point has been passed.
	}
	
	public void execute()
	{
		/// TODO: This call is crashing the code because the getPercentComplete
		/// is using an internal variable that does not exist at the moment of
		/// the call. The drive subsystem motion driver should be the owner
		/// of such things and we should ask it "are we there yet?" with 'there'
		/// being a number 0.0 to 1.0 representing completion state.
		/// Query should return false if we have not reached desired place (or passed it)
		/// and should also be false if initialization is incomplete. Returns
		/// true only if we are at or past or done.
		/*
		if (Robot.autonomousSubsystem.getPercentComplete() >  pathCompPercent)
		{
		Robot.elevatorSubsystem.holdPosition(requestedPosition);
		}
		*/
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
