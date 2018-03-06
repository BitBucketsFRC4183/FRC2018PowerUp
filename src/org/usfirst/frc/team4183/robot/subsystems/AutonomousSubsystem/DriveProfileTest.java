package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team4183.utils.*;

import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem.*;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.ClosedOut;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.ThroatHold;
/**
 *
 */
public class DriveProfileTest extends CommandGroup
{
	
	public DriveProfileTest()
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
		// Assume all subsystems have been commanded to Idle at initialization
		addSequential(new DriveProfile(PathPlans.testTrajectory0));	// Got from A to B
	}

}
