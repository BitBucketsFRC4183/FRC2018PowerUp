package org.usfirst.frc.team4183.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class RobotMap {

	public final static int PRIMARY_PID_LOOP  = 0; // Constants to support new Talon interface types
	public final static int CASCADED_PID_LOOP = 1; // That should have been enumerated rather than int
	public final static int CONTROLLER_TIMEOUT_MS = 100; // Default timeout to wait for configuration response
	
	// In some cases the status frames from motor controllers can be adjusted
	// Here we provide a range of values to provide uniformity
	// and avoid magic numbers in the software. Note that faster values (shorter periods) increases
	// the traffic on the associated bus (e.g., CAN)
	public final static int SUPER_HIGH_STATUS_FRAME_PERIOD_MS  =   5;	// CAUTION!
	public final static int HIGH_STATUS_FRAME_PERIOD_MS        =  10;	
	public final static int MEDIUM_HIGH_STATUS_FRAME_PERIOD_MS =  20;
	public final static int MEDIUM_STATUS_FRAME_PERIOD_MS      =  50;
	public final static int LOW_STATUS_FRAME_PERIOD_MS        = 100;

	
	//Motor Current Limits for the Wheel Shooter adjust later
	public static final double WHEEL_SHOOTER_MAX_CURRENT = 50;
	
	//Intake Max Current Limits for Faults
	public static final double INTAKE_MAX_CURRENT = 10;
	
	
	// Nominal value of wheels on this robot (should be measured at the wheels with greatest contact to ground, not just assumed)
	public static final double WHEEL_DIAMETER_INCHES = 6.0;
	public static final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * Math.PI);
	
	public static final double MINUMUM_MOTOR_CURR = 1.25;
	public static final double MOTOR_TEST_PERCENT = 0.5;

	//inches per revolution for Elevator
	public static final int INCH_EXTENSION_ROT = 200;
	
    //Ramp Subsystem Motor Ports
  	public final static int LEFT_RAMP_MOTOR_ID = 1;
  	public final static int RIGHT_RAMP_MOTOR_ID = 2;
  	
  	//Ramp Subsystem Servo Port
  	public final static int RAMP_RELEASE_SERVO_ID = 1;
	
	
	//DriveSubystem Motors ports
	public final static int LEFT_DRIVE_MOTOR_FRONT_ID  = 13;
	public final static int LEFT_DRIVE_MOTOR_REAR_ID   = 14;
	public final static int RIGHT_DRIVE_MOTOR_FRONT_ID = 11;
	public final static int RIGHT_DRIVE_MOTOR_REAR_ID  = 12;

	// DriveSubsystem Motor Directions
	// Assuming a single stage gearbox and motors mounted on
	// interior with axle pointing outward. Using right hand
	// rule for positive motor command yields the following:
	//		Left			Right
	//		INVERTED		not-inverted
	public final static boolean LEFT_DRIVE_MOTOR_INVERSION_FLAG = true;
	public final static boolean RIGHT_DRIVE_MOTOR_INVERSION_FLAG = false;
	
	// If positive controller command yields positive rotation and positive encoder speed
	// then the motor sensor phase should be left false. If the encoder reads a negative
	// value when commanding positive rotation (as designed for the mechanism) then the
	// sense should be inverted by setting the flag to true
	public final static boolean LEFT_DRIVE_MOTOR_SENSOR_PHASE = false;
	public final static boolean RIGHT_DRIVE_MOTOR_SENSOR_PHASE = false;
	
	public final static FeedbackDevice DRIVE_MOTOR_FEEDBACK_DEVICE = FeedbackDevice.QuadEncoder;
	
	// The left and right sides may not be precisely balanced in terms of
	// friction at really low speeds. We would like fine control to be balanced
	// so the neutral deadband is adjusted to determine when the motors start
	// moving on each side. This also prevents the motor from moving when
	// really small commands are passed through.
	//
	// The values are determined empirically by simply driving the motors slowly
	// until they first start to move on one side and not the other. Increase the
	// values until the desired response is achieved.
	public final static double LEFT_DRIVE_MOTOR_NEUTRAL_DEADBAND  = 0.003;
	public final static double RIGHT_DRIVE_MOTOR_NEUTRAL_DEADBAND = 0.007;
	
	
	// Cube Manipulation
	// Mutual exclusion on shooter or lifter types
	
	//IntakeSubsystem Motors ports
	public final static int INTAKE_MOTOR_LEFT_ID      = 5;
	public final static int INTAKE_MOTOR_RIGHT_ID      = 9;
	
	
	//public static final int SPRING_SHOOTER_MOTOR_A_ID = 7;
	//public static final int SPRING_SHOOTER_MOTOR_B_ID = 8;
	
	public final static int ELEVATOR_MOTOR_A_ID = 7;
	public final static int ELEVATOR_MOTOR_B_ID = 8;
	
	public final static int THROAT_MOTOR_A_ID = 3;
	public final static int THROAT_MOTOR_B_ID = 6;
	
	//Elevator Pneumatics for Gear Box
	public final static int ELEVATOR_PNEUMA_BRAKE_OPEN_CHANNEL = 2;
	public final static int ELEVATOR_PNEUMA_BRAKE_CLOSE_CHANNEL = 3;
	
	//ClimbSubystem TBD... may be pneumatics
	
	// Pneumatics
	public final static int INTAKE_PNEUMA_CLOSED_CHANNEL    = 1;
	public final static int INTAKE_PNEUMA_OPEN_CHANNEL      = 0;
	
	// Autonomous Constants
	public static final double DRIVESTRAIGHT_MIN_DRIVE = 0;
	public static final double TURNBY_MIN_DRIVE = 0;
	

}
