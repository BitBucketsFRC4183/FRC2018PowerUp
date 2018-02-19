package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Deployed is similar to IDLE but represents a non-disabled state relative to robot mode
 */
public class Deployed extends Command {

    public Deployed() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intakeSubsystem);	
    }

    protected void releaseSoftButtons()
    {
    	Robot.oi.sbtnIntakeThroat.release();
    	Robot.oi.sbtnOuttakeThroat.release();
    	Robot.oi.sbtnCloseMandible.release();    	
    }
    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	// At initial deployment disable the intake and stop pressing any soft buttons
    	Robot.intakeSubsystem.disable();
    	releaseSoftButtons();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    	if (Robot.oi.sbtnOpenMandible.get())
    	{
    		Robot.intakeSubsystem.openMandible();
    	}
    	else if (Robot.oi.sbtnCloseMandible.get())
    	{
    		Robot.intakeSubsystem.closeMandible();
    	}
    	Robot.elevatorSubsystem.disableThroat();
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
   // 	if( Robot.oi.btnIntake.get()) {
    	//	return CommandUtils.stateChange(this, new Intaking());
   // 	}
    	if( Robot.oi.btnOuttake.get()) {
    		return CommandUtils.stateChange(this, new Outtaking());
    	}
    	if( Robot.oi.btnCloseGate.get()) {
    		return CommandUtils.stateChange(this, new IntakingClose());
    	}
    	if ( Robot.oi.btnOpenGate.get()) {
    		return CommandUtils.stateChange(this, new IntakingOpen());
    	}
    	if (Robot.oi.btnRotateCube.get())
    	{
    		return CommandUtils.stateChange(this, new IntakeRotate());
    	}
    //	if( Robot.oi.btnCloseGate.get()) {
    	//	return CommandUtils.stateChange(this, new Idle());
    	//}
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
