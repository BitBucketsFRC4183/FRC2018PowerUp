package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4183.utils.*;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem.*;
import org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.RepositionAuto;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.UpShoot;
/**
 *
 */
public class AutoGameTasks extends CommandGroup
{
	
	public AutoGameTasks()
	{
		 // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
		RobotTrajectory trajectory = PathPlans.getSelectedTrajectory();
		
		// Assume all subsystems have been commanded to Idle at initialization
		/// TODO: Why does this not work? addParallel(new ThroatHold());
		if (trajectory != null)
		{
			// All trajectories can be delayed by a short amount if we need
			// to avoid an alliance member that has less control over their auto
			addSequential(new Delay((long) (1000 * SmartDashboard.getNumber("Auto Delay(sec)", 0))));
			
			// Add a parallel lift command to trigger at some percentage (e.g., 60%) of trajectory completion
			// NOTE: **** If **** this does not work correctly then we will need to use sequences follows:
			//		Drive Profile (Just Short of Scale)
			//		Raise Elevator
			//		MoveBy remaining difference
			if (trajectory.name.toLowerCase().contains("scale"))
			{
				//sets the elevator to this state which only moves the elevator to the high position if the the robot has completed 60% of the path
				addParallel(new RepositionAuto(ElevatorSubsystem.ElevatorPresets.HIGH.getNativeTicks(),.6));
			}
			else if (trajectory.name.toLowerCase().contains("switch"))
			{
				addParallel(new RepositionAuto(ElevatorSubsystem.ElevatorPresets.MIDDLE.getNativeTicks(),.25));
			}
			addSequential(new DriveProfile(trajectory));
						
			// Only spit the cube out if there is a scoring solution
			// Trajectories named "MoveOnly" or similar indicate that we should NOT eject the cube
			if (! trajectory.name.toLowerCase().contains("moveonly"))
			{
				addSequential(new UpShoot(1));
			}
			
			// TODO: If a two cube auto is desired then we would add the details here.
		}
		
		// Be explicit about getting the intake back into idle
		// NOTE: There are some problems with the ClosedOut actually completing execution
		// There appears to be an interrupted condition that causes the command to exit early
		// and prevents this auto sequence from completing correctly
		addSequential(new org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.DownHold());
	}

}
