package org.usfirst.frc.team4183.robot.subsystems.VisionSubsystem;

import org.usfirst.frc.team4183.robot.OI;
import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Idle extends Command {

	OI.ButtonEvent btnToggleCamMode;
	OI.ButtonEvent btnFrontCamMode;
	OI.ButtonEvent btnRearCamMode;

    public Idle() 
    {
        requires(Robot.visionSubsystem);
        setRunWhenDisabled(true);  // Required for Idle states!
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	System.out.println(this.getClass().getSimpleName());
//    	btnToggleCamMode = Robot.oi.getBtnEvt(Robot.oi.btnToggleFrontCameraView);
//    	btnFrontCamMode = Robot.oi.getBtnEvt(Robot.oi.btnSelectFrontCam);
//    	btnRearCamMode = Robot.oi.getBtnEvt(Robot.oi.btnSelectRearCam);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    /*if(btnToggleCamMode.onPressed()) 
    	{
    		if(Robot.visionSubsystem.isBoilerMode()) 
    		{
    			Robot.visionSubsystem.setGearMode();
    		}
    		else if(Robot.visionSubsystem.isGearMode()) 
    		{
    			Robot.visionSubsystem.setBoilerMode();
    		}
    	}
    	if(btnFrontCamMode.onPressed()) 
    	{
    		Robot.visionSubsystem.setFrontCam();
    	}
    	else if(btnRearCamMode.onPressed()) 
    	{
    		Robot.visionSubsystem.setRearCam();
    	}
    	
    	// These functions will only change things if necessary
    	Robot.visionSubsystem.updateTime();
    	Robot.visionSubsystem.setAllianceColor();
    	Robot.visionSubsystem.setAllianceNumber();
    	*/
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
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
