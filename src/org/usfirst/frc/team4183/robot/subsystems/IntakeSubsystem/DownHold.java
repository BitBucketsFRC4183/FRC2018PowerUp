package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.Robot.RunMode;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DownHold extends Command {

    public DownHold() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intakeSubsystem);	
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.intakeSubsystem.disable();	// Turn everything off and close it
    	Robot.intakeSubsystem.intakeDownPivet();
    	System.out.println(this.getClass().getSimpleName());
    	}

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.intakeSubsystem.setLeftThroatSpeed(RobotMap.THROAT_LEFT_HOLD_PERCENT);
    	Robot.intakeSubsystem.setRightThroatSpeed(RobotMap.THROAT_RIGHT_HOLD_PERCENT);
    	Robot.intakeSubsystem.setIntakeOnlySpeed(RobotMap.INTAKE_MOTOR_HOLD_PERCENT);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Robot.runMode == Robot.RunMode.TEST) {
    		if(Robot.intakeSubsystem.getDiagnosticsFlag()) {
    			return CommandUtils.stateChange(this, new Diagnostics());
    		}
    		
    	}
    	else if (Robot.oi.btnIdle.get())
    	{
    		return CommandUtils.stateChange(this, new Idle());
    	}
    	 else if (Robot.oi.btnUpIntake.get()) 
         { 
           return CommandUtils.stateChange(this, new UpHold()); 
         } 
         else if (Robot.oi.btnOutIntake.get()) 
         { 
           return CommandUtils.stateChange(this, new DownOut()); 
         }   
         else if (Robot.oi.btnInIntake.get()) 
         { 
           return CommandUtils.stateChange(this, new DownIn()); 
         } 
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.intakeSubsystem.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
