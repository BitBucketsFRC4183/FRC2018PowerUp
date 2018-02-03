package org.usfirst.frc.team4183.robot.subsystems.WheelShooterSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.CommandUtils;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Shooting extends Command 
{

    public Shooting() 
    {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.wheelShooterSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.oi.btnDriveLock.push();
    	Robot.wheelShooterSubsystem.setGateOpen();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.wheelShooterSubsystem.getFireSpeedPos() == WheelShooterSubsystem.FirePos.MANUAL)
    	{
    		Robot.wheelShooterSubsystem.setMotorSpeed(SmartDashboard.getNumber("Shooter Speed", 0));
    	}
    	else
    	{
    	Robot.wheelShooterSubsystem.setMotorSpeed(Robot.wheelShooterSubsystem.getFireSpeedPos().getPow());
    	}
    	
    	if (timeSinceInitialized()  > .5)
    	{
    		Robot.oi.btnIntake.push();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	if (Robot.wheelShooterSubsystem.getCurrentMax() > RobotMap.WHEEL_SHOOTER_MAX_CURRENT)
    	{
    		return CommandUtils.stateChange(this, new Fail());
    	}
    else if(Robot.oi.btnIdle.get() || (timeSinceInitialized()  > 2 && Robot.wheelShooterSubsystem.getFireSpeedPos() != WheelShooterSubsystem.FirePos.MANUAL)) {
    		return CommandUtils.stateChange(this, new Idle());
}
    	
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() 
    {    
    	Robot.wheelShooterSubsystem.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() 
    {
    	end();
    }
}
