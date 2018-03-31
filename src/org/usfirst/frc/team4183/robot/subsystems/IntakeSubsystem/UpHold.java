package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;
 
import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.Robot.RunMode;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.usfirst.frc.team4183.utils.CommandUtils;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
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
      System.out.println(this.getClass().getName() + " Start" + " " + System.currentTimeMillis()/1000);
      }
 
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		Robot.intakeSubsystem.setNeutral(NeutralMode.Brake);
    	Robot.intakeSubsystem.setLeftThroatSpeed(RobotMap.THROAT_LEFT_HOLD_PERCENT);
    	Robot.intakeSubsystem.setRightThroatSpeed(RobotMap.THROAT_RIGHT_HOLD_PERCENT);
    	
    	/*
    		IntakeBehavior when the the Elevator is moving:
    		if not in Dangerzone:
    			Checks to see if our velocity is positive i.e. the elevator is going up
    				Sets intake to zero power
    			Checks to see if our velocity is negative i.e. the elevator is going down
    				Sets intake to intake the cube.
    		if in Dangerzone:
    			Checks to see if our velocity is positive i.e. the elevator is going up
    				sets intake to push the cube out.
    			Checks to see if the our velocity is negative i.e. the elevator is going down
    				sets intake to intake the cube.
    	
    	*/
    	if (!Robot.elevatorSubsystem.outputDangerZoneInfo())
    	{
    		if (Robot.elevatorSubsystem.getCurrentSetTicks() > ElevatorSubsystem.ElevatorPresets.BOTTOM.getNativeTicks())
    		{
    		Robot.intakeSubsystem.setIntakeOnlySpeed(0);
    		}
    		else if (Robot.elevatorSubsystem.getCurrentSetTicks() < ElevatorSubsystem.ElevatorPresets.MIDDLE.getNativeTicks())
    		{
    			Robot.intakeSubsystem.setIntakeOnlySpeed(RobotMap.INTAKE_MOTOR_HOLD_PERCENT);
    		}
    	}
    	else if (Robot.elevatorSubsystem.outputDangerZoneInfo())
    	{
    		if (Robot.elevatorSubsystem.getCurrentSetTicks() > ElevatorSubsystem.ElevatorPresets.BOTTOM.getNativeTicks())
    		{

    			Robot.intakeSubsystem.setIntakeOnlySpeed(RobotMap.INTAKE_MOTOR_ASSIST_PERCENT);
    			//Robot.intakeSubsystem.setIntakeMotorsToSpeed(-RobotMap.INTAKE_MOTOR_HOLD_PERCENT, -RobotMap.INTAKE_MOTOR_HOLD_PERCENT);
    		}
    		else if (Robot.elevatorSubsystem.getCurrentSetTicks() < ElevatorSubsystem.ElevatorPresets.MIDDLE.getNativeTicks())
    		{
    			Robot.intakeSubsystem.setIntakeOnlySpeed(RobotMap.INTAKE_MOTOR_HOLD_PERCENT);
    			//Robot.intakeSubsystem.setIntakeMotorsToSpeed(RobotMap.INTAKE_MOTOR_HOLD_PERCENT, RobotMap.INTAKE_MOTOR_HOLD_PERCENT);
    		}
    	}
    	}
    


    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
      if (Robot.oi.btnInIntake.get())
      {
        return CommandUtils.autoStateChange(this, new ThroatPassOff());
      }
      else if (Robot.oi.btnDownIntake.get())
      {
        return CommandUtils.autoStateChange(this, new DownHold());
      }
      else if (Robot.oi.btnOutIntake.get() && !Robot.elevatorSubsystem.outputDangerZoneInfo())
      {
    	  return CommandUtils.autoStateChange(this, new UpShoot());
      }
      else if (Robot.oi.btnIdle.get())
      {
    	  return CommandUtils.autoStateChange(this, new Idle());
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
      end();
    }
}