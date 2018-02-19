package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.utils.CommandUtils;

import edu.wpi.first.wpilibj.command.Command;

public class Idle extends Command{

	public Idle()
	{
		requires(Robot.elevatorSubsystem);
		setRunWhenDisabled(true);
	}
	
	protected void initialize()
	{
		Robot.elevatorSubsystem.disable();
		//Robot.elevatorSubsystem.holdEncodPos(false);
		//Robot.elevatorSubsystem.holdEncodPos(true);
	}
	
	public void execute()
	{
		boolean in  = Robot.oi.sbtnIntakeThroat.get();
		boolean out = Robot.oi.sbtnOuttakeThroat.get();
		if (in ^ out) // Require mutual exclusion (i.e., cannot be both buttons at same time
		{
			if (in)
			{
				Robot.elevatorSubsystem.intakeThroat();
			}
			else if (out)
			{
				Robot.elevatorSubsystem.outtakeThroat();
			}
		}
		else // Could be both on or both off (either way, disable)
		{
			Robot.elevatorSubsystem.disableThroat();
			
			if (in && out)
			{
				System.out.println("ERROR: Both Throat In and Out Commanded!");
			}
		}
	}
	
	protected boolean isFinished()
	{ /*
		if (Robot.intakeSubsystem.getLastCurrent())
		{
			return CommandUtils.stateChange(this, new Loaded());
		}
		
		if (!Robot.intakeSubsystem.getLastCurrent())
		{
			return CommandUtils.stateChange(this, new Empty());
		}
		if (timeSinceInitialized() > Robot.elevatorSubsystem.timeUntilBrakeSec)
		{
			Robot.elevatorSubsystem.engageBrake();
			return CommandUtils.stateChange(this, new Brake());
		}
		*/
		/// Temporary code for testing linkage
		/// TODO: magic 0.06
		if (Math.abs(Robot.oi.leftRampAxis.get()) > .06)		/// TODO: Need comment here explaining that sharing joytick!
		{														/// TODO: May want two instances with different names and put comment at definition
			return CommandUtils.stateChange(this, new Reposition());
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
	
	/// TODO: Hidden state should be move to different file
	public class Empty extends Idle
	{

		public Empty()
		{
			//add the light change
			requires(Robot.elevatorSubsystem);
			Robot.elevatorSubsystem.holdEncodPos(true);
		}
		
		public void execute()
		{
			//Checks while in idle to see if the intake has called for the throat to spin.
			if (Robot.oi.sbtnIntakeThroat.get())
			{
				Robot.elevatorSubsystem.intakeThroat();
			}
			else if (Robot.oi.sbtnOuttakeThroat.get())
			{
				Robot.elevatorSubsystem.outtakeThroat();
			}
			else
			{
				Robot.elevatorSubsystem.disableThroat();
			}
		}
		
		@Override
		protected boolean isFinished() {
			 if (Math.abs(Robot.oi.leftRampAxis.get()) > .06 || Robot.oi.btnMedPosElev.get() || Robot.oi.btnHighPosElev.get() || Robot.oi.btnLowPosElev.get() || Robot.oi.btnTransPosElev.get())
				{
					return CommandUtils.stateChange(this, new Reposition());
				}
			 
			 if (timeSinceInitialized() > Robot.elevatorSubsystem.timeUntilBrakeSec)
				{
					return CommandUtils.stateChange(this, new Brake());
				}
			return false;
		}
		
	}
	public class Loaded extends Command
	{

		public Loaded()
		{
			//add light change
			requires(Robot.elevatorSubsystem);
			Robot.elevatorSubsystem.holdEncodPos(true);
		}
		
		public void execute()
		{
			if (Robot.oi.sbtnIntakeThroat.get())
			{
				Robot.elevatorSubsystem.intakeThroat();
			}
			else if (Robot.oi.sbtnOuttakeThroat.get())
			{
				Robot.elevatorSubsystem.outtakeThroat();
			}
			else
			{
				Robot.elevatorSubsystem.disableThroat();
			}
		}
		
		@Override
		protected boolean isFinished() {
			 if (Math.abs(Robot.oi.leftRampAxis.get()) > .06 || Robot.oi.btnMedPosElev.get() || Robot.oi.btnHighPosElev.get() || Robot.oi.btnLowPosElev.get() || Robot.oi.btnTransPosElev.get())
				{
					return CommandUtils.stateChange(this, new Reposition());
				}
			 
			 if (timeSinceInitialized() > Robot.elevatorSubsystem.timeUntilBrakeSec)
				{
					return CommandUtils.stateChange(this, new Brake());
				}
			return false;
		}
		
		protected void end()
		{
	
		}	
	
	}
	
	

}

