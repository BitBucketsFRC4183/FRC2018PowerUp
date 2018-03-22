package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;
 
import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.Robot.RunMode;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.usfirst.frc.team4183.utils.CommandUtils;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;
 
/**
 *
 */
public class UpHold extends Command {
 
    public UpHold() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
      requires(Robot.intakeSubsystem);  
    }
 
    // Called just before this Command runs the first time
    protected void initialize() 
    {
      Robot.intakeSubsystem.disable();  // Turn everything off and close it
      Robot.intakeSubsystem.intakeUpPivet();
      System.out.println(this.getClass().getSimpleName());
      }
 
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		Robot.intakeSubsystem.setNeutral(NeutralMode.Brake);
    	Robot.intakeSubsystem.setIntakeMotorsToSpeed(RobotMap.THROAT_LEFT_HOLD_PERCENT, RobotMap.THROAT_RIGHT_HOLD_PERCENT);
    	Robot.intakeSubsystem.setLeftThroatSpeed(RobotMap.THROAT_LEFT_HOLD_PERCENT);
    	Robot.intakeSubsystem.setRightThroatSpeed(RobotMap.THROAT_RIGHT_HOLD_PERCENT);
    	}
    


    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
      if (Robot.oi.btnInIntake.get())
      {
        return CommandUtils.stateChange(this, new ThroatPassOff());
      }
      else if (Robot.oi.btnDownIntake.get())
      {
        return CommandUtils.stateChange(this, new DownHold());
      }
      else if (Robot.oi.btnOutIntake.get() && !Robot.elevatorSubsystem.outputDangerZoneInfo())
      {
    	  return CommandUtils.stateChange(this, new UpShoot());
      }
      else if (Robot.oi.btnIdle.get() || Robot.elevatorSubsystem.getElevatorNativeUnits() > ElevatorSubsystem.ElevatorPresets.BOTTOM.getNativeTicks())
      {
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