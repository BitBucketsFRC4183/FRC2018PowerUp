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

	
	public static SendableChooser<SubsystemTelemetryState> telemetryState;
	private static ArrayList<Subsystem> subsystems;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	public HardwareStatusSubsystem() {
		subsystems = new ArrayList<Subsystem>();
		
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
	public boolean addSubsystemToStatusCheck(Subsystem subsystem) {
		if(!subsystems.contains(subsystem)) {
			subsystems.add(subsystem);
			return true;
		} else {
			return false;
		}
	}
	
	public void subsystemStatusCheck() {
		for(Subsystem system: subsystems) {
			
		}
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    @Override
    public void periodic() {
    	if(telemetryState.getSelected() == SubsystemTelemetryState.ON) {
    		SmartDashboard.putNumber("IMU_Yaw", 
    				Robot.imu.getYawDeg());
    		SmartDashboard.putNumber("IMU_Yawrate", 
    				Robot.imu.getYawRateDps());
    	}
    	
    }
}

