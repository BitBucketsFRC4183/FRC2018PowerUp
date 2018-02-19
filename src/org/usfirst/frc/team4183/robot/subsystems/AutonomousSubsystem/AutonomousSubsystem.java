package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import java.util.List;

import org.usfirst.frc.team4183.robot.subsystems.BitBucketsSubsystem;
import org.usfirst.frc.team4183.utils.Positions;
import org.usfirst.frc.team4183.utils.Positions.GenericPositions;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AutonomousSubsystem extends BitBucketsSubsystem 
{
	
	public AutonomousSubsystem()
	{						
		
		
	}

	public void initialize()
	{
		//for(int a=0; a<100; a++) System.out.println("?!?!?!??!?!??!?!?!");
		PathPlans.initialize();	

	}
	
	private static Positions.GenericPositions scalePosition;
	private static Positions.GenericPositions switchPosition;
	
	public void convertGameData()
	{
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		
		if (gameData.length() > 0)
		{
			if (gameData.charAt(0) == 'L')
			{
				switchPosition = GenericPositions.LEFT;
			}
			else
			{
				switchPosition = GenericPositions.RIGHT;
			}
			
			if(gameData.charAt(1)== 'L')
			{
				scalePosition = GenericPositions.LEFT;
			}
			else
			{
				scalePosition = GenericPositions.RIGHT;
			}
		}
	}
	
    public void initDefaultCommand() 
    {
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
	public void periodic() 
	{
		// TODO Insert processing to decide if re-planning is needed, and to raise the flag
		// Need to decide how to synchronize the plan updates (if needed) with the state processing.
		// it is possible that the re-plan is a state that is held until the replan flag is dropped.
		//
		// NOTE: If planning will take a long time, then it should be pushed to a separate thread
		// and monitored here, lowering the planning flag when the work is done. This keeps the
		// WPI scheduler running as expected (allowing the system to function within the parameters
		// established by the FMS, DriveStation, and the WPI architecture.
		
	}

	
}

