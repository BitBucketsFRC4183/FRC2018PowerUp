package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Diagnostics extends Command {
	
	private int diagInitLoops;

	
    public Diagnostics() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.elevatorSubsystem);
    	diagInitLoops = 0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevatorSubsystem.diagnosticsInit();
    	System.out.println(this.getClass().getName() + " Start" + " " + System.currentTimeMillis()/1000);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(this.timeSinceInitialized() > RobotMap.ELEVATOR_TEST_TIMEOUT_SEC) {
    		Robot.elevatorSubsystem.diagnosticsCheck();
    		return CommandUtils.stateChange(this, new Idle());
    	}
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println(this.getClass().getName() + " END" + " " + System.currentTimeMillis()/1000);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
