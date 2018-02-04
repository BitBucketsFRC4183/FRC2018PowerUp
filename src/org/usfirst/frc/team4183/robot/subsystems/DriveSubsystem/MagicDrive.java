package org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

public class MagicDrive {
	private SpeedController leftDrive;
	private SpeedController rightDrive;
	
	MagicDrive(SpeedController leftDriveParr, SpeedController rightDriveParr)
	{
		leftDrive = leftDriveParr;
		rightDrive = rightDriveParr;
	}
	
	enum Direction
	{
		FORWARD(1),REVERSE(-1);
		
		private final double powerDir;
		
		Direction(double pow)
		{
			this.powerDir = pow;
		}
		
		double getPowDir()
		{
			return powerDir;
		}
	}
	
	
	/***
	 * Magic Drive Method. Place this method into any team's code and if their motor controllers implement SpeedController this should allow them to drive by time
	 * @param pwr the power parameter
	 * @param time for the motors to run
	 * @param A enumeration which has the two values Forward and Backwards and returns -1 or 1 in order to flip the direction of movement
	 */
	public void magicDriveTime(double pwr, double timeRemain, Direction direction)
	{		
		pwr = Math.abs(pwr) * direction.getPowDir();
		if (Timer.getMatchTime() > timeRemain)
		{
			leftDrive.set(pwr);
			rightDrive.set(pwr);
		}
		else
		{
			leftDrive.set(0);
			rightDrive.set(0);
		}
	}
}
