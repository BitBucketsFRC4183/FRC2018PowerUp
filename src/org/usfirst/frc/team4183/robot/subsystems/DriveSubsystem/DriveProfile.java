package org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem.DriveProfile;
import org.usfirst.frc.team4183.utils.CommandUtils;
import org.usfirst.frc.team4183.utils.RobotTrajectory;

import com.ctre.phoenix.motion.SetValueMotionProfile;

import edu.wpi.first.wpilibj.command.Command;

public class DriveProfile extends Command {

	RobotTrajectory trajectory;
	
	double timeout_sec = 0.0;
	
    public DriveProfile(RobotTrajectory aTrajectory) 
    {
    	requires(Robot.autonomousSubsystem);
    	
    	trajectory = aTrajectory;
    	
    	// Timeout is set to 2 seconds beyond the estimated trajectory length based on the 
    	// sum of all delta-t values (assumed to be monotonic)
    	timeout_sec = 2.0 + trajectory.center.length() * trajectory.center.get(0).dt;
    }
    
    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	Robot.driveSubsystem.startTrajectory(trajectory);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    	Robot.driveSubsystem.motionProfileDriver.control();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
    	// Send back to idle when auto ends or profile is complete
    	/// TODO: go to next auto state when doing multiple actions
    	/// as either a command group or is it better to let idle and subsystem design what is next
    	boolean timeout = (timeSinceInitialized() > timeout_sec);
    	if ( (Robot.runMode != Robot.RunMode.AUTO) ||
    		 (Robot.driveSubsystem.motionProfileDriver.getSetValue() != SetValueMotionProfile.Enable) ||
    		 timeout
    	   )
    	{
    		if (timeout)
    		{
    			//System.out.println("WARNING: DriveProfile Timedout!");
    		}
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
    	end();
    }
}

