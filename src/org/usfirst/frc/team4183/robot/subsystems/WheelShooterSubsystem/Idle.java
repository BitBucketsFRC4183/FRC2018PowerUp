package org.usfirst.frc.team4183.robot.subsystems.WheelShooterSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;
import org.usfirst.frc.team4183.robot.OI;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Idle extends Command 
{

    public Idle() 
    {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.wheelShooterSubsystem);
    	setRunWhenDisabled(true);  // Idle state needs this!
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.wheelShooterSubsystem.disable();
    

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
      //System.out.println("Im Waiting");
    	if (Robot.oi.btnHighShot.get())
    	{
    		Robot.wheelShooterSubsystem.setFireSpeedState(WheelShooterSubsystem.FirePos.HIGHSHOT);
    	}
    	else if (Robot.oi.btnLowShot.get())
    	{
    		Robot.wheelShooterSubsystem.setFireSpeedState(WheelShooterSubsystem.FirePos.LOWSHOT);
    	}
    	else if (Robot.oi.btnShooter.get())
    	{
    		Robot.wheelShooterSubsystem.setFireSpeedState(WheelShooterSubsystem.FirePos.MANUAL);
    	}

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	if(Robot.oi.shooterLoadAxis.get() > .5) {
    		return CommandUtils.stateChange(this, new Arm());	    	
    	}
    	
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() 
    {    
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() 
    {
    	end();
    }
}
