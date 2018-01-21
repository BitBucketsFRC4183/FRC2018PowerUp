package org.usfirst.frc.team4183.robot.commands.SpringShooterSubsystem;

import org.usfirst.frc.team4183.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Fire extends Command {

    public Fire() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    		requires(Robot.springShooterSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		//Flash red lights once
		//Release brake
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    		//Should return true as soon as brake is released
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    		//Goes to IDLE
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		end();
    }
}
