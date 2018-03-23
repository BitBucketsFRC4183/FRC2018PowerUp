package cheesecakeDrive;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/***
 * This class is type of cheesecaking that we give to all teams in the competition that don't have an auto.
 * @author rommac100
 */


public class CheesecakeDrive 
{
	private SpeedController leftDrive;
	private SpeedController rightDrive;
	
	private Timer driveTimer;

	CheesecakeDrive(SpeedController leftDriveParr, SpeedController rightDriveParr)
	{
		leftDrive = leftDriveParr;
		rightDrive = rightDriveParr;
		driveTimer = new Timer();
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
	public void drive(double pwr, double stopTime_seconds, Direction direction)
	{				
		// Keep commanding the motors until the timer reaches the stop time
		while (driveTimer.get() < stopTime_seconds)
		{
			if (direction == Direction.REVERSE)
			{
				pwr = -pwr;
			}
			leftDrive.set(pwr);
			rightDrive.set(pwr);
		}
		
		leftDrive.set(0);
		rightDrive.set(0);
	}
}
