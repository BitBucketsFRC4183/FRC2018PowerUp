package MagicDrive;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/***
 * This class is type of cheesecaking that we give to all teams in the competition that don't have an auto.
 * If they are on our alliance on any particular match then the auto will run otherwise it won't run so we don't benefit them too much
 * @author rommac100
 */


public class MagicDrive {
	private SpeedController leftDrive;
	private SpeedController rightDrive;

	public class MatchInfo
	{
		private int matchNum;
	
		private Alliance alliance;
		
		MatchInfo(int aMatchNum, Alliance aAlliance)
		{
			matchNum = aMatchNum;
			alliance = aAlliance;
		}
		
		protected int getMatchNum()
		{
			return matchNum;
		}
		
		protected Alliance getAllianceColour()
		{
			return alliance;
		}
	}
	
	public MatchInfo[] activeMatches = {new MatchInfo(20,Alliance.Red), new MatchInfo(30,Alliance.Blue)};
	
	
	
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
	
	public MatchInfo convertMatchInfo()
	{
		return new MatchInfo(DriverStation.getInstance().getMatchNumber(), DriverStation.getInstance().getAlliance());
	}
	
	//TRY this to see if the using a timer instead of match time works instead
	/***
	 * Magic Drive Method. Place this method into any team's code and if their motor controllers implement SpeedController this should allow them to drive by time
	 * @param pwr the power parameter
	 * @param time for the motors to run
	 * @param A enumeration which has the two values Forward and Backwards and returns -1 or 1 in order to flip the direction of movement
	 */
	public void magicDriveTime(double pwr, double timeRemain, Direction direction)
	{		
		//This is the check that sees if this alliance member is actually on our alliance that match. Else do not run this auto.
		boolean run = false;
		for (int i =0; i < activeMatches.length; i++)
		{
			if (convertMatchInfo() == activeMatches[i])
			{
				run = true;
				break;
			}
		}
		if (run)
		{
		pwr = Math.abs(pwr) * direction.getPowDir();
		Timer timerMagic = new Timer();
		timerMagic.reset();
		timerMagic.start();
		while (timerMagic.get() > timeRemain)
		{
			leftDrive.set(pwr);
			rightDrive.set(pwr);
		}
			leftDrive.set(0);
			rightDrive.set(0);
		}
		else
		{
			System.out.println("sorry you are not on our alliance");
		}
	}
}
