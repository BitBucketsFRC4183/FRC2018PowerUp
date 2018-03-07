package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team4183.utils.*;

import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem.*;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.ClosedOut;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.ThroatHold;
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
		//addParallel(new ThroatHold());
		if (trajectory != null)
		{
			/// TODO: consider a parallel command to pre-lift to mid height while driving
			/// to the scale, if selected. The rest of the lift will be commanded at the end
			addSequential(new DriveProfile(trajectory));	// Got from A to B
			/// TODO: add command to lift to height corresponding to trajectory choice
			addSequential(new ClosedOut(1.0));				// Spit out the cube at the end
		}
		else // good for just running into the scale
		{
			
			// Default to move forward in no more than 3 seconds
			/// TODO: Consider making the distance a robot constant, a chooser, or a preference
			addSequential(new MoveBy(85.0, 3.0));			// Need correct distance to just get to fence + a little
		}	
		addSequential(new org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.Idle());
	}

}
