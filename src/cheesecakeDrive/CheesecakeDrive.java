package cheesecakeDrive;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

/***
 * This class is type of cheesecaking that we give to all teams in the competition that don't have an auto.
 * @author rommac100
 */

public class CheesecakeDrive 
{
	
	private SpeedController leftSpeedController;
	private SpeedController rightSpeedController;

	private Timer driveTimer;
	
	
	/// TODO: Add a delay timer and a delay argument to drive
	/// Will have to change initialization sequence to start the delay timer
	/// and move the drive timer reset/start to the expired delay timer
	/// branch inside drive

	public CheesecakeDrive(SpeedController leftDriveParr, SpeedController rightDriveParr)
	{
		leftSpeedController = leftDriveParr;
		rightSpeedController = rightDriveParr;
		driveTimer = new Timer();
	}
		
	/***
	 * initialize - call in autonomousInit or whenever the timer needs to be reset
	 */
	public void initialize()
	{
		driveTimer.reset();
		driveTimer.start();
	}
	
	/***
	 * drive. 
	 * Place this method into any team's autonomousPeriodic code and if their motor controllers implement 
	 * SpeedController this should allow them to drive by time
	 * @param pwr the power parameter
	 * @param stopTime_sec - the number of seconds to run the motors
	 * @param A enumeration which has the two values Forward and Backwards and returns -1 or 1 in order to flip the direction of movement
	 */
	public void drive(double pwr, double stopTime_seconds)
	{				
		// Keep commanding the motors until the timer reaches the stop time
		if (driveTimer.get() < stopTime_seconds)
		{
			leftSpeedController.set(pwr);
			rightSpeedController.set(pwr);
		}
		else
		{
			leftSpeedController.set(0);
			rightSpeedController.set(0);
		}
	}
}
