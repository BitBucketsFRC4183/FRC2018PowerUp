package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.Robot.RunMode;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DownOff extends Command {

    public DownOff() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intakeSubsystem);	
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.intakeSubsystem.disable();	// Turn everything off and close it
    	Robot.intakeSubsystem.intakeDownPivet();
    	System.out.println(this.getClass().getName() + " Start" + " " + System.currentTimeMillis()/1000);
    	}

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Robot.runMode == Robot.RunMode.TEST) {
    		if(Robot.intakeSubsystem.getDiagnosticsFlag()) {
    			return CommandUtils.stateChange(this, new Diagnostics());
    		}
    		
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
         else if (Robot.oi.btnLeftIntake.get())
         {
        	 return CommandUtils.stateChange(this, new DownOscillate());
         }
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println(this.getClass().getName() + " End" + " " + System.currentTimeMillis()/1000);
    	Robot.intakeSubsystem.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
