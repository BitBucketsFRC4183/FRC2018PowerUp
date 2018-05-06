package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4183.utils.*;
import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem.*;
import org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem.ElevatorPresets;
import org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.Reposition;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.DownHold;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.DownIn;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.UpHold;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.UpShoot;
/**
 *
 */
public class AutoGameTasks extends CommandGroup
{
	
	public AutoGameTasks(double delay_sec)
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
			long delay = (long) (1000 * SmartDashboard.getNumber("Auto Delay(sec)", 0)-(1000*delay_sec));
			if (delay < 0)
			{
				delay = 0;
			}
			addSequential(new Delay(delay));
			
			// Add a parallel lift command to trigger at some percentage (e.g., 60%) of trajectory completion
			// NOTE: **** If **** this does not work correctly then we will need to use sequences follows:
			//		Drive Profile (Just Short of Scale)
			//		Raise Elevator
			//		MoveBy remaining difference
			addParallel(new UpHold());
			if(!trajectory.name.toLowerCase().contains("moveonly"))
			{
				if (trajectory.name.toLowerCase().contains("scale"))
				{
					//sets the elevator to this state which only moves the elevator to the high position if the the robot has completed 60% of the path
					addParallel(new Reposition(ElevatorSubsystem.ElevatorPresets.TOP.getNativeTicks(),0.75));
				}
				else if (trajectory.name.toLowerCase().contains("switch"))
				{
					addParallel(new Reposition(ElevatorSubsystem.ElevatorPresets.MIDDLE.getNativeTicks(),0.6));
				}
			}
			addSequential(new DriveProfile(trajectory));
						
			// Only spit the cube out if there is a scoring solution
			// Trajectories named "MoveOnly" or similar indicate that we should NOT eject the cube
			if (!trajectory.name.toLowerCase().contains("moveonly"))
			{
				addSequential(new UpShoot(0.5));
				if(trajectory.name.toLowerCase().contains("switch")) {
					if(trajectory.name.toLowerCase().contains("left")) {
						AutoCommandGroup.SecondCubeGroup(this, 63.0, 56.0, 55.0/*45.0*/, ElevatorSubsystem.ElevatorPresets.MIDDLE.getNativeTicks(), 3.0, false);
//						AutoCommandGroup.SecondCubeGroup(this, 60.0, 34.5, 43.5/*37.5*/, ElevatorSubsystem.ElevatorPresets.MIDDLE.getNativeTicks(), 3.0, false);
					}
					
					else if(trajectory.name.toLowerCase().contains("right"))// the right one
					{
						AutoCommandGroup.SecondCubeGroup(this, -63.0, 52.0, 55.0, ElevatorSubsystem.ElevatorPresets.MIDDLE.getNativeTicks(), 3.0, false);
//						AutoCommandGroup.SecondCubeGroup(this, -60.0, 34.5, 37.5, ElevatorSubsystem.ElevatorPresets.MIDDLE.getNativeTicks(), 3.0, false);
					}
				} 
				else if(trajectory.name.toLowerCase().contains("scale")) {
					addSequential(new MoveBy(-24.0, 2.0));
					addSequential(new Reposition(ElevatorPresets.BOTTOM.getNativeTicks()));
//				
					if(trajectory.name.toLowerCase().contains("left")) {
//						AutoCommandGroup.SecondCubeGroup(this, 104.183, 24.0, 59.0, ElevatorSubsystem.ElevatorPresets.TOP.getNativeTicks(), 0.0, true);
//						AutoCommandGroup.SecondCubeGroup(this,      88, 24.0, 76.5, ElevatorSubsystem.ElevatorPresets.TOP.getNativeTicks(), 0.0, true);
					}
					
					else if(trajectory.name.toLowerCase().contains("right"))// the right one
					{
//						AutoCommandGroup.SecondCubeGroup(this, -104.183, 24.0, 59.0, ElevatorSubsystem.ElevatorPresets.TOP.getNativeTicks(), 0.0, true);
//						AutoCommandGroup.SecondCubeGroup(this,    -88.0, 24.0, 76.5, ElevatorSubsystem.ElevatorPresets.TOP.getNativeTicks(), 0.0, true);
					}
				}

				addParallel(new org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.Idle());
				
			}
		
		}
		
		// Be explicit about getting the intake back into idle
		// NOTE: There are some problems with the ClosedOut actually completing execution
		// There appears to be an interrupted condition that causes the command to exit early
		// and prevents this auto sequence from completing correctly
		
	}

}
