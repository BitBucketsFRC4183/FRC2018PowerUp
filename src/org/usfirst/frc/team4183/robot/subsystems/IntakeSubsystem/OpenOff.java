package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class OpenOff extends Command {

    public OpenOff() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intakeSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intakeSubsystem.openMandible();
    	Robot.intakeSubsystem.setIntakeMotorSpeed(0.0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Robot.oi.btnIdle.get() || Robot.oi.btnCloseGate.get())
    		return CommandUtils.stateChange(this, new Idle());
    	else if(Robot.oi.btnInIntake.get())
    		return CommandUtils.stateChange(this, new OpenIn());
    	else if(Robot.oi.btnOutIntake.get())
    		return CommandUtils.stateChange(this, new OpenOut());
    	else if(Robot.oi.btnLeftIntake.get())
    		return CommandUtils.stateChange(this, new OpenLeft());
    	else if(Robot.oi.btnRightIntake.get())
    		return CommandUtils.stateChange(this, new OpenRight());
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
