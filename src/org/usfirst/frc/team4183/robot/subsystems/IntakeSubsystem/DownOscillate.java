package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;
 
import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.Robot.RunMode;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.CommandUtils;
 
import edu.wpi.first.wpilibj.command.Command;
 
/**
 *
 */
public class DownOscillate extends Command {
 
	private boolean oscilate = true;
	
    public DownOscillate() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
      requires(Robot.intakeSubsystem);  
    }
 
    // Called just before this Command runs the first time
    protected void initialize() 
    {
      Robot.intakeSubsystem.intakeDownPivet();
      System.out.println(this.getClass().getName() + " Start" + " " + System.currentTimeMillis()/1000);
      }
 
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (oscilate = true)
    	{
    		Robot.intakeSubsystem.setLeftIntakeSpeed(-RobotMap.INTAKE_OSCILLATE_MOTOR_PERCENT);
    		Robot.intakeSubsystem.setRightIntakeSpeed(RobotMap.INTAKE_OSCILLATE_MOTOR_PERCENT);
    	}
    	else if (oscilate = false)
    	{
    		Robot.intakeSubsystem.setLeftIntakeSpeed(-RobotMap.INTAKE_MOTOR_PERCENT);
    		Robot.intakeSubsystem.setRightIntakeSpeed(-RobotMap.INTAKE_MOTOR_PERCENT);
    	}
    	oscilate = !oscilate;
    }
 
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
      if (!Robot.oi.btnLeftIntake.get())
      {
        return CommandUtils.stateChange(this, new DownHold());
      }
      else if (Robot.oi.btnIdle.get())
      {
    	  return CommandUtils.stateChange(this, new DownOff());
      }
      
      return false;
    }
 
    // Called once after isFinished returns true
    protected void end() {
      Robot.intakeSubsystem.disable();
      System.out.println(this.getClass().getName() + " End" + " " + System.currentTimeMillis()/1000);
    }
 
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
      end();
    }
}