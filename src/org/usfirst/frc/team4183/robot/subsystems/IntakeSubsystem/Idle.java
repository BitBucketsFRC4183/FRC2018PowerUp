package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.Robot.RunMode;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Idle extends Command {

    public Idle() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intakeSubsystem);	
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.intakeSubsystem.disable();	// Turn everything off and close it
    	System.out.println(this.getClass().getSimpleName());
    	}

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Robot.runMode == Robot.RunMode.TEST) {
    		return CommandUtils.stateChange(this, new Diagnostics());
    	}
    	else if(Robot.oi.btnOpenGate.get() || Robot.oi.sbtnOpenMandible.get())
    		return CommandUtils.autoStateChange(this, new OpenOff());
    	else if(Robot.oi.btnOutIntake.get()|| Robot.oi.sbtnOuttakeThroat.get()) {
    		return CommandUtils.autoStateChange(this, new ClosedOut());
    	}
    	else if(Robot.oi.btnInIntake.get()) {
    		return CommandUtils.autoStateChange(this, new ClosedIn());
    	}
    	else if(Robot.oi.btnLeftIntake.get()) {
    		return CommandUtils.autoStateChange(this, new ClosedLeft());
    	}
    	else if(Robot.oi.btnRightIntake.get()) {
    		return CommandUtils.autoStateChange(this, new ClosedRight());
    	}
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
