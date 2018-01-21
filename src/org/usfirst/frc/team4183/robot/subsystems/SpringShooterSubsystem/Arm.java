package org.usfirst.frc.team4183.robot.subsystems.SpringShooterSubsystem;

import org.usfirst.frc.team4183.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Arm extends Command {

    public Arm() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    		requires(Robot.springShooterSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		//Run motor until platform is at its lowest position
		//Reload brake
		//Set gearbox to neutral (pneumatics)
		//Turn lights red
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    		//Returns true when FIRE button is pressed
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    		//Goes to FIRE
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    		end();
    }
}
