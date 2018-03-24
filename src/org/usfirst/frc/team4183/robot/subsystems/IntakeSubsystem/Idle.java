package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.Robot.RunMode;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.CommandUtils;

import com.ctre.phoenix.motorcontrol.NeutralMode;

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
    	Robot.intakeSubsystem.setNeutral(NeutralMode.Coast);
    	Robot.intakeSubsystem.disable();	// Turn everything off and close it
    	Robot.intakeSubsystem.intakeUpPivet();
    	System.out.println(this.getClass().getName() + "INTAKE START " + System.currentTimeMillis()/1000);
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
    	 else if (Robot.oi.btnDownIntake.get())
    	 {
    		 return CommandUtils.stateChange(this, new DownOff());
    	 }
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println(this.getClass().getName() + " INTAKE END" + " " + System.currentTimeMillis()/1000);
    	Robot.intakeSubsystem.setNeutral(NeutralMode.Brake);
    	Robot.intakeSubsystem.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
