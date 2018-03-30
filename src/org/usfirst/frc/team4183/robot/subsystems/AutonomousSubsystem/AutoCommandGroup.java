package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem.MoveBy;
import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem.TurnBy;
import org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.Reposition;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.DownHold;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.DownIn;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.UpHold;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.UpShoot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoCommandGroup{

    public static void SecondCubeGroup(CommandGroup parent, double turn_degree, double backup_in, double forward_in, int elevator_position, double final_offset, boolean backup_first) {
    	if(backup_first) 
    	{
        	parent.addSequential(new MoveBy(-backup_in, 2.0));

    	}
    	else if(!backup_first) 
    	{
    		parent.addParallel(new MoveBy(-backup_in, 2.0));
    	}
		parent.addSequential(new Reposition(ElevatorSubsystem.ElevatorPresets.BOTTOM.getNativeTicks()));
		parent.addParallel(new org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.Idle());
		parent.addParallel(new DownIn());
		parent.addSequential(new TurnBy(turn_degree, 1.0)); 
		parent.addSequential(new MoveBy(forward_in, 2.0));
		parent.addSequential(new Delay(1000));
		parent.addParallel(new DownHold());
		parent.addSequential(new MoveBy(-forward_in, 2.0));
		parent.addParallel(new UpHold());
		parent.addSequential(new Delay(300));
		parent.addSequential(new TurnBy(-turn_degree, 1.0));
		parent.addParallel(new Reposition(elevator_position));
		parent.addSequential(new MoveBy(backup_in + final_offset, 1.5));
		parent.addSequential(new UpShoot(RobotMap.SHORT_SHOT_SEC));
    	
    }
}
