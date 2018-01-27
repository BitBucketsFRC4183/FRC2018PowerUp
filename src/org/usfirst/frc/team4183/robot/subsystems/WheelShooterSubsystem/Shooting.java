package org.usfirst.frc.team4183.robot.subsystems.WheelShooterSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
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
    	setRunWhenDisabled(true);  // Idle state needs this!
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	
    

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.wheelShooterSubsystem.setMotorSpeed(SmartDashboard.getNumber("Shooter Speed", 0));
    //	Robot.wheelShooterSubsystem.setMotorSpeed(Robot.oi.wheelShooterAxis.get());
    //System.out.println("Im Shooting");

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	/// TODO: Resurrect this from last year
    	if(Robot.oi.btnIdle.get()) {
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
