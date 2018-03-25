package org.usfirst.frc.team4183.robot.subsystems.ClimberSubsystem;

import org.usfirst.frc.team4183.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Climb extends Command {

    public Climb() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.climberSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println(this.getClass().getName() + " Start" + " " + System.currentTimeMillis()/1000);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double x = Robot.oi.leftRampAxis.get();
    	x *= x;
    	if (x > 0.06)
    	{
    		Robot.climberSubsystem.setClimberPower(x);
    	}
    	else
    	{
    		Robot.climberSubsystem.setClimberPower(0);	
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println(this.getClass().getName() + " Start" + " " + System.currentTimeMillis()/1000);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
