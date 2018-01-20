package org.usfirst.frc.team4183.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4183.robot.subsystems.SubsystemUtilities.SubsystemTelemetryState;

/**
 *
 */
public abstract class BitBucketsSubsystem extends Subsystem {
	
	// Add global fields here (Motors, constants, choosers, etc) 
	private static SendableChooser<SubsystemTelemetryState> telemetryState;
	
	public BitBucketsSubsystem() {
		telemetryState = new SendableChooser<SubsystemTelemetryState>();
		
		telemetryState.addDefault("Off", SubsystemTelemetryState.OFF);
		telemetryState.addObject( "On",  SubsystemTelemetryState.ON);
		
		SmartDashboard.putData("Example Subsystem Telemetry", telemetryState);
	}
	
	/* Any hardware devices used in this subsystem must
	*  have a check here to see if it is still connected and 
	*  working properly. For motors check for current draw.
	*  Return true iff all devices are working properly. Otherwise
	*  return false.
	*/
	public abstract boolean diagnostics();
	
	
	@Override
    protected abstract void initDefaultCommand();
    
    // Runs every 20ms. Put debug information here
    @Override
    public void periodic() {
    	
    	if(telemetryState.getSelected() == SubsystemTelemetryState.ON) {
    		// Put telemetry information here
    	}
    }
}

