package org.usfirst.frc.team4183.robot.subsystems.WheelShooterSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;
import org.usfirst.frc.team4183.robot.OI;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Arm extends Command 
{

    public Arm() 
    {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.wheelShooterSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
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
    	
    	if (Robot.wheelShooterSubsystem.getFireSpeedPos() == WheelShooterSubsystem.FirePos.MANUAL)
    	{
    		Robot.wheelShooterSubsystem.setMotorSpeed(SmartDashboard.getNumber("Shooter Speed", 0));
    	}
    	else
    	{
    	Robot.wheelShooterSubsystem.setMotorSpeed(Robot.wheelShooterSubsystem.getFireSpeedPos().getPow());
    	}
    	}

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	// TODO:  WHEN ENCODERS ARE IMPLEMENTED ADDED THE CASE THAT WAITS FOR THE WHEELS TO BE AT DESIRED SPEED
    	if(Robot.oi.shooterFireAxis.get() > .5) {
    		return CommandUtils.stateChange(this, new Shooting());
        }
    	
    	if (Robot.oi.btnIdle.get())
    	{
    		return CommandUtils.stateChange(this, new Idle());
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
