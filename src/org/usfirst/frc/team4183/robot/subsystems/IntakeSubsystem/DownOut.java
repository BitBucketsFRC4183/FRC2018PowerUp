package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;
 
import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.Robot.RunMode;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.CommandUtils;
 
import edu.wpi.first.wpilibj.command.Command;
 
/**
 *
 */
public class DownOut extends Command {
 
    public DownOut() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
      requires(Robot.intakeSubsystem);  
    }
 
    // Called just before this Command runs the first time
    protected void initialize() 
    {
      //Double check that the intake is down
      Robot.intakeSubsystem.intakeDownPivet();
      System.out.println(this.getClass().getSimpleName());
      }
 
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    
    	  Robot.intakeSubsystem.setIntakeMotorsToSpeed(RobotMap.INTAKE_MOTOR_PERCENT, RobotMap.INTAKE_MOTOR_PERCENT);
    }
 
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
      if (Robot.oi.btnIdle.get() || Robot.oi.btnOpenGate.get() || !Robot.oi.btnOutIntake.get())
      {
        return CommandUtils.stateChange(this, new Idle());
      }
      
      return false;
    }
 
    // Called once after isFinished returns true
    protected void end() {
      Robot.intakeSubsystem.disable();
    }
 
    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
      end();
    }
}