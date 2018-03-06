package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class OpenRight extends Command {

    public OpenRight() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intakeSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intakeSubsystem.openMandible();
    	System.out.println(this.getClass().getSimpleName());
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.intakeSubsystem.setLeftMotorSpeed(RobotMap.INTAKE_MOTOR_PERCENT, RobotMap.THROAT_MOTOR_PERCENT);
    	Robot.intakeSubsystem.setRightMotorSpeed(-RobotMap.INTAKE_MOTOR_PERCENT, -RobotMap.THROAT_MOTOR_PERCENT);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(Robot.oi.btnIdle.get())
    		return CommandUtils.autoStateChange(this, new Idle());
    	else if (! Robot.oi.btnRightIntake.get())
    		return CommandUtils.autoStateChange(this, new OpenOff());
    	else if(Robot.oi.btnCloseGate.get())
    		return CommandUtils.autoStateChange(this, new ClosedIn());
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
