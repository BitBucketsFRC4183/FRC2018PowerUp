package org.usfirst.frc.team4183.robot;

public class RobotMap {
	
	// Nominal value assuming 4" wheel:
	// (4" * pi) in/rot = 12.57
	// The precise value must be determined by calibration.
	public static final double INCH_PER_WHEEL_ROT = 12.5 * 1.0353360615;
	
	// Constant as UPPERCASE_WITH_UNDERSCORES
	
	
	//DriveSubystem Motors ports
	public final static int LEFT_DRIVE_MOTOR_FRONT_ID  = 1;
	public final static int LEFT_DRIVE_MOTOR_BACK_ID   = 2;
	public final static int RIGHT_DRIVE_MOTOR_FRONT_ID = 3;
	public final static int RIGHT_DRIVE_MOTOR_BACK_ID  = 4;
	
	//ClimbSubystem Motors ports
	public final static int CLIMB_MOTOR_LEFT_ID        = 5;
	public final static int CLIMB_MOTOR_RIGHT_ID       = 6;

}
