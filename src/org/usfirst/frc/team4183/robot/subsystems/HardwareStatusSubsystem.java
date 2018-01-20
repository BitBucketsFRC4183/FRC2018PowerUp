package org.usfirst.frc.team4183.robot.subsystems;

import java.util.ArrayList;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.subsystems.SubsystemUtilities.SubsystemTelemetryState;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class HardwareStatusSubsystem extends Subsystem {

	private static int numDiagnosticsLoops = 0;
	
	public static SendableChooser<SubsystemTelemetryState> telemetryState;
	private static ArrayList<BitBucketsSubsystem> subsystemsList;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public HardwareStatusSubsystem() {
		subsystemsList = new ArrayList<BitBucketsSubsystem>();
		
		telemetryState = new SendableChooser<SubsystemTelemetryState>();
    	
    	telemetryState.addDefault("Off", SubsystemTelemetryState.OFF);
    	telemetryState.addObject( "On",  SubsystemTelemetryState.ON);
    	
    	SmartDashboard.putData("HardwareTelemetry", telemetryState);
	}
	
	/**
	 * This will add a subsystem to the debug list as long as it's 
	 * not already in the list. Returns true if successfully added 
	 * 
	 */
	public boolean addSubsystemToDiagnostics(BitBucketsSubsystem subsystem) {
		if(!subsystemsList.contains(subsystem)) {
			subsystemsList.add(subsystem);
			return true;
		} else {
			return false;
		}
	}
	
	/**
	 * Removes subsystem from diagnostics if in the list. Returns true if
	 * successfully removed, false otherwise.
	 * 
	 */
	public boolean removeSubsystemFromDiagnostic(BitBucketsSubsystem subsystem) {
		if(subsystemsList.contains(subsystem)) {
			subsystemsList.remove(subsystem);
			return true;
		} else {
			return false;
		}
	}
	
	public void subsystemStatusCheck() {
		for(BitBucketsSubsystem subsystem: subsystemsList) {
			subsystem.diagnosticsCheck();
		}
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    @Override
    public void periodic() {
    	// subsystemStatusCheck();
    	
    	if(Robot.runMode == Robot.RunMode.TEST ) {
    		SmartDashboard.putBoolean("RunningDiag", true);
    		numDiagnosticsLoops++;
    		for(BitBucketsSubsystem subsystem: subsystemsList)
    			subsystem.diagnosticsInit();
    	}
    	else if (Robot.runMode == Robot.RunMode.TEST && numDiagnosticsLoops >= 5) {
    		numDiagnosticsLoops = 0;
    		for(BitBucketsSubsystem subsystem: subsystemsList)
    			subsystem.diagnosticsCheck();
    	}
    	
    	if(telemetryState.getSelected() == SubsystemTelemetryState.ON) {
    		SmartDashboard.putNumber("IMU_Yaw", 
    				Robot.imu.getYawDeg());
    		SmartDashboard.putNumber("IMU_Yawrate", 
    				Robot.imu.getYawRateDps());
    	}
    	
    }
}

