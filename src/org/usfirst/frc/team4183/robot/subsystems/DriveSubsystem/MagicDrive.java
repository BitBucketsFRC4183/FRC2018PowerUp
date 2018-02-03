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
	
	
	/***
	 * Magic Drive Method. Place this method into any team's code and if their motor controllers implement SpeedController this should allow them to drive by time
	 * @param pwr the power parameter
	 * @param time for the motors to run
	 * @param forward a boolean that determines direction, True for forward or positive direction, False for reverse or negative direction.
	 */
	public void magicDriveTime(double pwr, double timeRemain, boolean forward)
	{
		double dir = 1;
		if (!forward)
		{
			dir = -1;
		}
			
		pwr = Math.abs(pwr);
		if (Timer.getMatchTime() > timeRemain)
		{
			leftDrive.set(pwr*dir);
			rightDrive.set(pwr*dir);
		}
		else
		{
			leftDrive.set(0);
			rightDrive.set(0);
		}
	}
}
