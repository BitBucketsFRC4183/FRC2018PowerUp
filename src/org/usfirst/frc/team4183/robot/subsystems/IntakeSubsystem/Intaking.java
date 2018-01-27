package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Intaking extends Command {

    public Intaking() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    		requires(Robot.intakeSubsystem);	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    //	Robot.intakeSubsystem.setMotorSpeed(SmartDashboard.getNumber("Shooting Speed", 0));    
    	Robot.intakeSubsystem.setMotorSpeed(-0.5);    

    	}

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if( Robot.oi.btnIdle.get()) {
    		return CommandUtils.stateChange(this, new Idle());
    	}
    	if( Robot.oi.btnOuttake.get()) {
    		return CommandUtils.stateChange(this, new Outtaking());
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
