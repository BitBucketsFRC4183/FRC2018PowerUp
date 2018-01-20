package org.usfirst.frc.team4183.robot;

public class RobotMap {
	
	// Nominal value assuming 4" wheel:
	// (4" * pi) in/rot = 12.57
	// The precise value must be determined by calibration.
	public static final double INCH_PER_WHEEL_ROT = 12.5 * 1.0353360615;
	
	//inches per revolution for Elevator
    public static final int INCH_EXTENSION_ROT = 200;
	
	// Constant as UPPERCASE_WITH_UNDERSCORES
	
	
	//DriveSubystem Motors ports
	public final static int LEFT_DRIVE_MOTOR_FRONT_ID  = 3;
	public final static int LEFT_DRIVE_MOTOR_REAR_ID   = 4;
	public final static int RIGHT_DRIVE_MOTOR_FRONT_ID = 1;
	public final static int RIGHT_DRIVE_MOTOR_REAR_ID  = 2;
	
	//Elevator Motor Ports
	public final static int ELEVATOR_MOTOR_A_ID = 5;
	public final static int ELEVATOR_MOTOR_B_ID = 6;
	
	//ClimbSubystem Motors ports
	//public final static int CLIMB_MOTOR_LEFT_ID        = 5;
	//public final static int CLIMB_MOTOR_RIGHT_ID       = 6;
	
	//IntakeSubsystem Motors ports
	public final static int INTAKE_MOTOR_LEFT_ID      = 6;
	public final static int INTAKE_MOTOR_RIGHT_ID      = 5;
	
	public final static int WHEEL_SHOOTER_LEFT_1_MOTOR_ID     = 7;
	public final static int WHEEL_SHOOTER_LEFT_2_MOTOR_ID     = 8;
	public final static int WHEEL_SHOOTER_RIGHT_1_MOTOR_ID    = 9;
	public final static int WHEEL_SHOOTER_RIGHT_2_MOTOR_ID    = 10;
	// Pneumatics
	public final static int INTAKE_PNEUMA_CLOSED_CHANNEL    = 1;
	public final static int INTAKE_PNEUMA_OPEN_CHANNEL      = 0;
}
