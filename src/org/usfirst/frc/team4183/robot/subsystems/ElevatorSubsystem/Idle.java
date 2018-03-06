package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;
import org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem.ElevatorPresets;

import edu.wpi.first.wpilibj.command.Command;

public class Idle extends Command{

	private int holdPosition = 0;
	
	public Idle()
	{
		requires(Robot.elevatorSubsystem);
		setRunWhenDisabled(true);
	}
	
	protected void initialize()
	{
		Robot.elevatorSubsystem.disable();
		Robot.oi.sbtnOpenMandible.release();
		
		holdPosition = (int) Robot.elevatorSubsystem.getElevatorNativeUnits();
		System.out.println(this.getClass().getSimpleName());
		
		//Robot.elevatorSubsystem.holdEncodPos(false);
		//Robot.elevatorSubsystem.holdEncodPos(true);
	}
	
	public void execute()
	{
		Robot.elevatorSubsystem.holdPosition(holdPosition);
	}
	
	protected boolean isFinished()
	{
		/// Temporary code for testing linkage
		/// TODO: magic 0.06
		if(Robot.runMode == Robot.RunMode.TEST) {
			if(Robot.elevatorSubsystem.getDiagnosticsFlag()) {
				return CommandUtils.stateChange(this, new Diagnostics());
			}
		}
		
		else if (Math.abs(Robot.oi.rightRampAxis.get()) > .06)		/// TODO: Need comment here explaining that sharing joytick!
		{														/// TODO: May want two instances with different names and put comment at definition
			Robot.oi.sbtnOpenMandible.push();
			return CommandUtils.stateChange(this, new Reposition());
		}
		else if (Robot.oi.btnElevatorTop.get())
		{
			Robot.oi.sbtnOpenMandible.push();
			return CommandUtils.stateChange(this, new Reposition(ElevatorPresets.TOP.getNativeTicks()));			
		}
		else if (Robot.oi.btnElevatorHigh.get())
		{
			Robot.oi.sbtnOpenMandible.push();
			return CommandUtils.stateChange(this, new Reposition(ElevatorPresets.HIGH.getNativeTicks()));			
		}
		else if (Robot.oi.btnElevatorMiddle.get())
		{
			Robot.oi.sbtnOpenMandible.push();
			return CommandUtils.stateChange(this, new Reposition(ElevatorPresets.MIDDLE.getNativeTicks()));			
		}
		else if (Robot.oi.btnElevatorBottom.get())
		{
			Robot.oi.sbtnOpenMandible.push();
			return CommandUtils.stateChange(this, new Reposition(ElevatorPresets.BOTTOM.getNativeTicks()));			
		}
		
		
		return false;
	}

	protected void end()
	{
		
	}
	
	protected void interrupted()
	{
		end();
	}
	
}
