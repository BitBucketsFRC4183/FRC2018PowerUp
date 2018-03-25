package org.usfirst.frc.team4183.robot.subsystems.ClimberSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Idle extends Command {

    public Idle() {
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
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Robot.getTeleopTimeRemaining() <= 30.00 || SmartDashboard.getNumber("ClimberTimeOverride", 0) == 141367) {
    		return CommandUtils.stateChange(this, new Climb());
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
    }
}
