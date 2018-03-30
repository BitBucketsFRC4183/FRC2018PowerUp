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
    	Robot.intakeSubsystem.setLeftThroatSpeed(RobotMap.THROAT_LEFT_HOLD_PERCENT);
    	Robot.intakeSubsystem.setRightThroatSpeed(RobotMap.THROAT_RIGHT_HOLD_PERCENT);
    	Robot.intakeSubsystem.setIntakeOnlySpeed(RobotMap.INTAKE_MOTOR_HOLD_PERCENT);
    	System.out.println(this.getClass().getName() + " Start" + " " + System.currentTimeMillis()/1000);
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
    			return CommandUtils.autoStateChange(this, new Diagnostics());
    		}
    		
    	}
    	else if (Robot.oi.btnIdle.get())
    	{
    		return CommandUtils.autoStateChange(this, new Idle());
    	}
    	 else if (Robot.oi.btnUpIntake.get()) 
         { 
           return CommandUtils.autoStateChange(this, new UpHold()); 
         } 
         else if (Robot.oi.btnOutIntake.get()) 
         { 
           return CommandUtils.autoStateChange(this, new DownOut()); 
         }   
         else if (Robot.oi.btnInIntake.get()) 
         { 
           return CommandUtils.autoStateChange(this, new DownIn()); 
         }
         else if (Robot.oi.btnLeftIntake.get())
         {
        	 return CommandUtils.autoStateChange(this, new DownOscillate());
         }
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println(this.getClass().getName() + " END" + " " + System.currentTimeMillis()/1000);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
