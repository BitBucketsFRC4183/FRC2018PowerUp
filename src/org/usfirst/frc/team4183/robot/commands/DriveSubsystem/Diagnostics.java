package org.usfirst.frc.team4183.robot.commands.DriveSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.subsystems.BitBucketsSubsystem;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Diagnostics extends Command {

	private int diagInitLoops;
	private BitBucketsSubsystem subsystem;
	
    public Diagnostics() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveSubsystem);
    	subsystem = Robot.driveSubsystem;
    	diagInitLoops = 0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	SmartDashboard.putBoolean("EnterDiagState", true);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(diagInitLoops < subsystem.DIAG_LOOPS_RUN) {
    		subsystem.diagnosticsInit();
    		diagInitLoops++;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(diagInitLoops >= subsystem.DIAG_LOOPS_RUN) {
    		subsystem.diagnosticsCheck();
    		return CommandUtils.stateChange(this, new Idle());
    	}
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
