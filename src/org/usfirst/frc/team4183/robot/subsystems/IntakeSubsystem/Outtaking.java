package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Outtaking extends Command {

    public Outtaking() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    		requires(Robot.intakeSubsystem);	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    //	Robot.intakeSubsystem.opengate();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.intakeSubsystem.setMotorSpeed(-1);    
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if( Robot.oi.btnIdle.get()) {
    		return CommandUtils.stateChange(this, new Deployed());
    	}
    	if ( Robot.oi.btnIntake.get()) {
    		return CommandUtils.stateChange(this, new Intaking());
    	}
    		
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
