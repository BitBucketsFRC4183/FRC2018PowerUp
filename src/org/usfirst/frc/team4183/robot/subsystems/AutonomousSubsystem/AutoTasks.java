package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem.*;
/**
 *
 */
public class AutoTasks extends CommandGroup {

    public AutoTasks() {
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
    	
    	addSequential(new MoveBy(24, 3));
    	addSequential(new TurnBy(500, 3));
    	addSequential(new MoveBy(-24, 3));
    	
    }
}
