package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Deployed extends Command {

    public Deployed() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    		requires(Robot.intakeSubsystem);	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intakeSubsystem.disable();
    	Robot.oi.sbtnOuttakeThroat.release();
    	Robot.oi.sbtnCloseMandible.release();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.oi.sbtnOpenMandible.get())
    	{
    		Robot.intakeSubsystem.opengate();
    	}
    	else if (Robot.oi.sbtnCloseMandible.get())
    	{
    		Robot.intakeSubsystem.closegate();
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
