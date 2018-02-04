package org.usfirst.frc.team4183.robot;

public class RobotMap {

	public final static int PRIMARY_PID_LOOP  = 0; // Constants to support new Talon interface types
	public final static int CASCADED_PID_LOOP = 1; // That should have been enumerated rather than int
	public final static int CONTROLLER_TIMEOUT_MS = 100; // Default timeout to wait for configuration response

	
	//Motor Current Limits for the Wheel Shooter adjust later
	public static final double WHEEL_SHOOTER_MAX_CURRENT = 50;
	
	//Intake Max Current Limits for Faults
	public static final double INTAKE_MAX_CURRENT = 10;
	
	// Nominal value assuming 4" wheel:
	// (4" * pi) in/rot = 12.57
	// The precise value must be determined by calibration.
	public static final double INCH_PER_WHEEL_ROT = 12.5 * 1.0353360615;
	
	public static final double MINUMUM_MOTOR_CURR = 1.25;
	public static final double MOTOR_TEST_PERCENT = 0.5;

  //inches per revolution for Elevator
  public static final int INCH_EXTENSION_ROT = 200;
	
    //Ramp Subsystem Motor Ports
  	public final static int LEFT_RAMP_MOTOR_ID = 7;
  	public final static int RIGHT_RAMP_MOTOR_ID = 8;
  	
  	//Ramp Subsystem Servo Port
  	public final static int RAMP_RELEASE_SERVO_ID = 1;
	
	
	//DriveSubystem Motors ports
	public final static int LEFT_DRIVE_MOTOR_FRONT_ID  = 3;
	public final static int LEFT_DRIVE_MOTOR_REAR_ID   = 4;
	public final static int RIGHT_DRIVE_MOTOR_FRONT_ID = 7;
	public final static int RIGHT_DRIVE_MOTOR_REAR_ID  = 8;

	// Cube Manipulation
	// Mutual exclusion on shooter or lifter types
	
	//IntakeSubsystem Motors ports
	public final static int INTAKE_MOTOR_LEFT_ID      = 5;
	public final static int INTAKE_MOTOR_RIGHT_ID      = 9;
	
	// Wheel shooter, spring shooter, and elevator are mutually exlusive
	public final static int WHEEL_SHOOTER_LEFT_1_MOTOR_ID     = 5;
	public final static int WHEEL_SHOOTER_RIGHT_1_MOTOR_ID    = 11;
	
	//public static final int SPRING_SHOOTER_MOTOR_A_ID = 7;
	//public static final int SPRING_SHOOTER_MOTOR_B_ID = 8;
	
	public final static int ELEVATOR_MOTOR_A_ID = 7;
	public final static int ELEVATOR_MOTOR_B_ID = 8;
	
	//ClimbSubystem TBD... may be pneumatics
	
	// Pneumatics
	public final static int INTAKE_PNEUMA_CLOSED_CHANNEL    = 1;
	public final static int INTAKE_PNEUMA_OPEN_CHANNEL      = 0;
	public final static int WHEEL_SHOOTER_HIGH_POS_CHANNEL = 2;
	public final static int WHEEL_SHOOTER_LOW_POS_CHANNEL = 3;
	public final static int GATE_OPEN_POS_CHANNEL = 4;
	public final static int GATE_CLOSE_POS_CHANNEL = 5;
	
	// Autonomous Constants
	public static final double DRIVESTRAIGHT_MIN_DRIVE = 0;
	public static final double TURNBY_MIN_DRIVE = 0;
	

}
