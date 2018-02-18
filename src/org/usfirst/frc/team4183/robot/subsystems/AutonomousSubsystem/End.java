package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import org.usfirst.frc.team4183.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * End state - for testing
 */
public class End extends Command {

    public End() {
    	requires( Robot.autonomousSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.oi.axisForward.set(0.0);
    	Robot.oi.axisTurn.set(0.0);
    	Robot.oi.btnIdle.hit();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	// No exit from this state
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
