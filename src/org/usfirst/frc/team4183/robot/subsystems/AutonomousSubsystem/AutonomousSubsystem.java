package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import java.util.List;

import org.usfirst.frc.team4183.robot.subsystems.BitBucketsSubsystem;
import org.usfirst.frc.team4183.utils.Positions;
import org.usfirst.frc.team4183.utils.Positions.GenericPositions;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AutonomousSubsystem extends BitBucketsSubsystem 
{
	private static boolean planning = true;	// Start this way and lower the flag when it is okay to let states begin
											// NOTE: This is addition to the robot mode check because it also allows
											// a replanning state to be injected and held in Idle
		
	public static boolean isPlanning()
	{
		return planning;
	}
	
	// This class is unique per Robot and Game
	// We need selectors for autonomy modes (including test modes)
	// and then a way of kicking it off when commanded from the Robot
	// passing through autonomousInit
	
	private static SendableChooser<Positions.StartingPosition> startingPosition;

	// Is this a re-hash of the CommandGroup concept or is it more
	// ordered and state like? The fact that a re-plan can interject
	// into the command conditions means that the grouping is not
	// as rigid in the future. I.e., if the transition paths are
	// restricted by our concept of a state machine, then CommandGroups
	// are not very useful to producing a well behaved sequence that
	// is also flexible to a certain boundary.
	//
	// NOT SURE YET... this may just be a semantics issue with the Command type
	public enum AutoTasks // For this robot and game
	{
		STANDBY,	// Wait for further instruction
		
		// Basic sequences
		MOVE_BY,			// Use Motion Magic or similar means to produce precise moves forward or backward
		TURN_BY,			// Use MM etc for precise turns
		SET_ELEVATOR_TO,
		OPEN_INTAKE,
		CLOSE_INTAKE,
		
		DRIVE_PROFILE,	// Use a pre-planned or on-the-fly Motion Profile
		LOOK_FOR_EXCHANGE,
		LOOK_FOR_CUBE,
		DRIVE_TO_CUBE,
		INTAKE_CUBE,
		EXCHANGE_CUBE,
		LIFT_CUBE_FOR_SWITCH,
		LIFT_CUBE_FOR_SCALE_LOW,
		LIFT_CUBE_FOR_SCALE_HIGH,
		EJECT_CUBE
		
	};
	public class AutoTaskDescriptor
	{
		public AutoTasks task;
		public double     value; // Interpreted by task. Could be inches, degrees, etc. may be a problem when using
//		DriveProfile	profile;
		
		public AutoTaskDescriptor()
		{
			// Default task descriptor is to do nothing
			task = AutoTasks.STANDBY;
			value = 0.0;
		}
	};
	
	private static List<AutoTaskDescriptor> plan;	// A place to inject the plan tokens
	// Nothing in here now...it's pretty much a dummy,
	// just to make the state machine work.
	
	private static AutoTaskDescriptor currentDriveTask;
	
	public AutonomousSubsystem()
	{						
		currentDriveTask = new AutoTaskDescriptor();
		
		startingPosition = new SendableChooser<Positions.StartingPosition>();
		startingPosition.addDefault("Center", Positions.StartingPosition.CENTER);
		startingPosition.addObject("Left", Positions.StartingPosition.LEFT);
		startingPosition.addObject("Right", Positions.StartingPosition.RIGHT);
		
		SmartDashboard.putData("StartingPosition", startingPosition);
		
		
	}

	public void initialize()
	{
		PathPlans.initialize();		
	}
	
	private static Positions.GenericPositions scalePosition;
	private static Positions.GenericPositions switchPosition;
	
	public void convertGameData()
	{
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		
		if (gameData.length() > 0)
		{
			if (gameData.charAt(0) == 'L')
			{
				switchPosition = GenericPositions.LEFT;
			}
			else
			{
				switchPosition = GenericPositions.RIGHT;
			}
			
			if(gameData.charAt(1)== 'L')
			{
				scalePosition = GenericPositions.LEFT;
			}
			else
			{
				scalePosition = GenericPositions.RIGHT;
			}
		}
	}
	
    public void initDefaultCommand() 
    {
        setDefaultCommand(new Idle());
    }

	@Override
	public void diagnosticsInit() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void diagnosticsExecute() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void diagnosticsCheck() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void setDiagnosticsFlag(boolean enable) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public boolean getDiagnosticsFlag() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void periodic() 
	{
		// TODO Insert processing to decide if re-planning is needed, and to raise the flag
		// Need to decide how to synchronize the plan updates (if needed) with the state processing.
		// it is possible that the re-plan is a state that is held until the replan flag is dropped.
		//
		// NOTE: If planning will take a long time, then it should be pushed to a separate thread
		// and monitored here, lowering the planning flag when the work is done. This keeps the
		// WPI scheduler running as expected (allowing the system to function within the parameters
		// established by the FMS, DriveStation, and the WPI architecture.
		
	}

	public static void setDriveTask(AutoTasks aTask, double aValue) 
	{
		if (currentDriveTask.task == AutoTasks.STANDBY)
		{
			currentDriveTask.value = aValue;
			currentDriveTask.task = aTask;
		}
	}

	public static AutoTaskDescriptor getDriveTask() 
	{
		// TODO Auto-generated method stub
		return currentDriveTask;
	}

	public static void setDriveTaskComplete(boolean aComplete) 
	{
		if (aComplete)
		{
			// When something signals complete, just go to standby
			// to let the autonomy state advance
			currentDriveTask.task = AutoTasks.STANDBY;
			currentDriveTask.value = 0.0;
		}
		
	}

	public static AutoTaskDescriptor getNextTask() 
	{
		// TODO Some kind of list processor where list can be cleared
		// and rebuilt as needed
		AutoTaskDescriptor nextTask = null;
		if (! plan.isEmpty())
		{
			nextTask = plan.get(0);
			plan.remove(0);
		}
		
		return nextTask;
	}
}

