package org.usfirst.frc.team4183.robot.subsystems;

public class SubsystemUtilities {
	
	public enum SubsystemTelemetryState 
	{
		OFF,
		ON
	}
	
	public enum DiagnosticsInformation 
	{
		SUBSYSTEM_BASIC,
		SUBSYSTEM_EXTENDED
	}
	
	public enum BITMode
	{
		INIT, 
		EXTENDED
	}

	public enum DiagnosticsState 
	{ 
		UNKNOWN,
		PASS,
		FAIL
	}
}
