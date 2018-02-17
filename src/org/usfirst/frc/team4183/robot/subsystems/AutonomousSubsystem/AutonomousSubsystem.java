package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import org.usfirst.frc.team4183.robot.subsystems.BitBucketsSubsystem;
import org.usfirst.frc.team4183.utils.Positions;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AutonomousSubsystem extends BitBucketsSubsystem {

	// Nothing in here now...it's pretty much a dummy,
	// just to make the state machine work.
	
	private static SendableChooser<Positions.StartingPosition> startingPosition;

	private static Positions.GenericPositions scalePositon;
	private static Positions.GenericPositions switchPosition;
	
	
	public AutonomousSubsystem()
	{
		startingPosition = new SendableChooser<Positions.StartingPosition>();
		startingPosition.addDefault("Center", Positions.StartingPosition.CENTER);
		startingPosition.addObject("LEFT", Positions.StartingPosition.LEFT);
		startingPosition.addObject("Right", Positions.StartingPosition.RIGHT);
		
		SmartDashboard.putData(startingPosition);
		
	}
	
	public void convertGameData()
	{
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		if (gameData.length() > 0)
		{
			if (gameData.charAt(0) == 'L')
			{
				
			}
			else if (gameData.charAt(0) == 'R') 
			{
				
			}
		}
	}
	
    public void initDefaultCommand() {
        setDefaultCommand(new Idle());
    }

	@Override
	public void diagnosticsInit() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void diagnosticsExecute() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void diagnosticsCheck() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void setDiagnosticsFlag(boolean enable) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public boolean getDiagnosticsFlag() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void periodic() {
		// TODO Auto-generated method stub
		
	}
}

