package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;
 
import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.Robot.RunMode;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.CommandUtils;
 
import edu.wpi.first.wpilibj.command.Command;
 
/**
 *
 */
public class DownIn extends Command {
 
    public DownIn() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
      requires(Robot.intakeSubsystem);  
    }
 
    // Called just before this Command runs the first time
    protected void initialize() 
    {
      Robot.intakeSubsystem.intakeDownPivet();
      Robot.intakeSubsystem.setIntakeOnlySpeed(-RobotMap.INTAKE_MOTOR_PERCENT);
      System.out.println(this.getClass().getName() + " Start" + " " + System.currentTimeMillis()/1000);
      }
 
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {   
    	Robot.intakeSubsystem.setIntakeOnlySpeed(-RobotMap.INTAKE_MOTOR_PERCENT);
    	//Robot.intakeSubsystem.setThroatSpeed(-RobotMap.THROAT_LEFT_HOLD_PERCENT);
    }
 
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
      if (Robot.oi.btnOutIntake.get())
      {
        return CommandUtils.autoStateChange(this, new ThroatPassOff());
      }
      else if (!Robot.oi.btnInIntake.get() && Robot.runMode != Robot.RunMode.AUTO)
      {
        return CommandUtils.autoStateChange(this, new DownHold());
      }
      else if (Robot.oi.btnUpIntake.get())
      {
    	  return CommandUtils.autoStateChange(this, new UpHold());
      }
      else if (Robot.oi.btnIdle.get())
      {
    	  return CommandUtils.autoStateChange(this, new Idle());
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