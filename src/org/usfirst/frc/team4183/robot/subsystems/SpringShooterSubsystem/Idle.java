package org.usfirst.frc.team4183.robot.subsystems.SpringShooterSubsystem;
import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 *
 */
public class Idle extends Command 
{
    public Idle() {
        // Use requires() here to declare subsystem dependencies
    		requires(Robot.springShooterSubsystem);
    		setRunWhenDisabled(true);  // Idle state needs this!
    }
    // Called just before this Command runs the first time
    protected void initialize() {
    		Robot.springShooterSubsystem.disable();
    		//Set gearbox to high torque (pneumatics)
    		//Release brake
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		//Check for cube on platform
    }
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    		//Returns true when cube is detected
    	if (Robot.springShooterSubsystem.isPresent())
    	{
    	}
    		if(Robot.runMode==Robot.RunMode.TEST && Robot.springShooterSubsystem.runDiagnostics==true) 
    			return CommandUtils.stateChange(this, new Diagnostics());
    		
    		return false;
    }
    // Called once after isFinished returns true
    	protected void end() {
    		SmartDashboard.putBoolean("EnteringDiagnostics", true);

    		//Goes to LOADED
    }
    	// Called when another command which requires one or more of the same
    	// subsystems is scheduled to run
    	protected void interrupted() {
    		end();
    }
}