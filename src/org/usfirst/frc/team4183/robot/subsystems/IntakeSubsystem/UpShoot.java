package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;
 
import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.Robot.RunMode;
import org.usfirst.frc.team4183.utils.CommandUtils;
 
import edu.wpi.first.wpilibj.command.Command;
 
/**
 *
 */
public class UpShoot extends Command {
 
  private double timeout_sec = 0;
  
    public UpShoot(double aTimeOut_sec) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
      requires(Robot.intakeSubsystem);  
      timeout_sec = aTimeOut_sec;
    }
    
    public UpShoot() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
      requires(Robot.intakeSubsystem);  
    }
    
    // Called just before this Command runs the first time
    protected void initialize() 
    {
      Robot.intakeSubsystem.setIntakeOnlySpeed(0);
      Robot.intakeSubsystem.intakeUpPivet();
      System.out.println(this.getClass().getName() + " Start" + " " + System.currentTimeMillis()/1000);
      }
 
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
      Robot.intakeSubsystem.setLeftThroatSpeed(RobotMap.INTAKE_MOTOR_PERCENT);
      Robot.intakeSubsystem.setRightThroatSpeed(RobotMap.INTAKE_MOTOR_PERCENT);
    }
 
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
      if (timeout_sec == 0)
      {
	      if (!Robot.oi.btnOutIntake.get())
	      {
	        return CommandUtils.autoStateChange(this, new UpHold());
	      }
	      if (Robot.oi.btnDownIntake.get())
	      {
	        return CommandUtils.autoStateChange(this, new DownHold());
	      }
	      else if (Robot.oi.btnIdle.get())
	      {
	    	  return CommandUtils.autoStateChange(this, new Idle());
	      }
      }
      else
      {
        if (timeSinceInitialized() > timeout_sec)
        {
          return CommandUtils.autoStateChange(this, new UpHold());
        }
      }
      
      return false;
    }
 
    // Called once after isFinished returns true
    protected void end() {
      System.out.println(this.getClass().getName() + " END" + " " + System.currentTimeMillis()/1000);
      Robot.intakeSubsystem.disable();
    }
 
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
      end();
    }
}
 