package org.usfirst.frc.team4183.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class RobotMap 
{
	public static double inch2Meter(double inch)
	{
		return 0.3048 * inch / 12.0; 
	}
	public static double meter2inch(double meter)
	{
		return 12.0 * meter / 0.3048; 
	}
	
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

	/**
	 * Intake Subsystem Limit Switch Mappings
	 */
	public final static int INTAKE_LIMIT_LEFT_MAX_ID = 0;
	public final static int INTAKE_LIMIT_LEFT_MIN_ID = 1;
	public final static int INTAKE_LIMIT_RIGHT_MAX_ID = 2;
	public final static int INTAKE_LIMIT_RIGHT_MIN_ID = 3;
	
	
	// Intake motor speed
	public static final double INTAKE_MOTOR_PERCENT = 0.8;
	
	//Motor Current Limits for the Wheel Shooter adjust later
	public static final double WHEEL_SHOOTER_MAX_CURRENT = 50;
	
	//Intake Max Current Limits for Faults
	public static final double INTAKE_MAX_CURRENT = 10;
	
	
	// Nominal value of wheels on this robot (measured from edge to edge of the blue nitrile tire)
	public static final double WHEEL_DIAMETER_INCHES = 6.25;
	public static final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * Math.PI);

	public static final double MAXIMUM_MOTION_ERROR_INCHES = 0.125;	// Convert into native ticks later

	
	public static final double MINUMUM_MOTOR_CURR = 1.25; 
	public static final double MOTOR_TEST_PERCENT = 0.5; 	
	
	// Wheel track measured from inside edge to edge of blue nitrile tires in center position of robot
	public static final double WHEEL_TRACK_INCHES = 24.25;
	
	// Ratio of circumference to track determined the turn angle
	public static final double TRACK_TO_CIRCUMFERENCE_RATIO = WHEEL_TRACK_INCHES / WHEEL_DIAMETER_INCHES;
	
	public static final double WHEEL_ROTATION_PER_FRAME_DEGREES = TRACK_TO_CIRCUMFERENCE_RATIO / 360.0;
	
	//inches per revolution for Elevator
	//Nominal diameter to the pin
	public static final double ELEVATOR_SPROCKET_DIAMETER_INCHES  = 3;
	public static final double ELEVATOR_SPROCKET_CIRCUMFERENCE_INCHES = (ELEVATOR_SPROCKET_DIAMETER_INCHES*Math.PI);
	
	//Magic Motion Constants for the Elevator Subsystem
	public final static boolean ELEVATOR_MOTOR_SENSOR_PHASE = false;
		
	public final static FeedbackDevice ELEVATOR_MOTOR_FEEDBACK_DEVICE = FeedbackDevice.QuadEncoder;
	public final static int ELEVATOR_MOTOR_NATIVE_TICKS_PER_REV = 8192;
	public final static double ELEVATOR_MOTOR_FULL_THROTTLE_AVERAGE_SPEED_NATIVE_TICKS = 25588.4;	// per 100 ms, average of 10 samples
	
	public final static int ELEVATOR_MOTOR_MOTION_CRUISE_SPEED_NATIVE_TICKS = (int)(0.80 * 
                                                                              ELEVATOR_MOTOR_FULL_THROTTLE_AVERAGE_SPEED_NATIVE_TICKS);

	public final static int ELEVATOR_MOTOR_MOTION_ACCELERATION_NATIVE_TICKS = ELEVATOR_MOTOR_MOTION_CRUISE_SPEED_NATIVE_TICKS;
	
	//Deadband defines when motion can start (i.e., minimum input required)
	public final static double ELEVATOR_MOTOR_NEUTRAL_DEADBAND  = 0.000; //ADJUST
	
	public static final double ELEVATOR_INCHES_PER_NATIVE_TICKS = ELEVATOR_SPROCKET_CIRCUMFERENCE_INCHES / ELEVATOR_MOTOR_NATIVE_TICKS_PER_REV;

	public static final double ELEVATOR_POSITION_TOLERANCE_INCH = 0.25;	/// TODO: What is possible and what do we want?
	public static final int ELEVATOR_POSITION_TOLERANCE_NATIVE_TICKS = (int) (ELEVATOR_POSITION_TOLERANCE_INCH / ELEVATOR_INCHES_PER_NATIVE_TICKS); //50;	/// TODO: Convert from inches or meter tolerance
	
	// The magic number 1023 is in the SRM based on the characteristics of the TalonSRX
	// It is likely based on the internal workings of the A-to-D conversions, but the details
	// are not important at this point; just consider it a scaling factor to make the numbers
	// work for the specific controllers we have.
	//
	// NOTE: These values were found empirically running the Mini-CIM on a 9:1 Versa-Planetary gear box (NO LOAD)
	// Followup testing will adjust these values based on the weight of the elevator
	// It is possible that the constants will be different for up and down, and even across the first stage transition.
	// We only get two sets per Talon and probably cannot switch the value fast enough to handling the
	// transition, so will need to be satisfied with just and up/down set if it comes down (or up) to that.
	public static double elevatorMotorKf = 0.111544836;
	public static double elevatorMotorKp = 0.1778688524;		
	public static double elevatorMotorKi = 0.001;
	public static double elevatorMotorKd = 5.336065573;
	public static int    elevatorMotorIZone = 300;

	
    //Ramp Subsystem Motor Ports
  	public final static int LEFT_RAMP_MOTOR_ID = 1;
  	public final static int RIGHT_RAMP_MOTOR_ID = 2;
  	
  	//Ramp Subsystem Servo Port
  	public final static int RAMP_RELEASE_SERVO_ID = 1;
	
	
	//DriveSubystem Motors ports
	public final static int LEFT_DRIVE_MOTOR_FRONT_ID  = 11; //13;
	public final static int LEFT_DRIVE_MOTOR_REAR_ID   = 12; //14;
	public final static int RIGHT_DRIVE_MOTOR_FRONT_ID = 13; //11;
	public final static int RIGHT_DRIVE_MOTOR_REAR_ID  = 14; //12;

	
	//IntakeSubsystem Motors ports
	public final static int INTAKE_MOTOR_LEFT_ID      = 10;
	public final static int INTAKE_MOTOR_RIGHT_ID      = 9;
	
	
	public final static int ELEVATOR_MOTOR_A_ID = 4;
	
	//A is left, B Right
	public final static int THROAT_MOTOR_LEFT_ID = 5;
	public final static int THROAT_MOTOR_RIGHT_ID = 3;

	
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
	public final static int DRIVE_MOTOR_NATIVE_TICKS_PER_REV = 8192;	// AMT-201 at 2048 pulses per rev
	
	public static final int DRIVE_MOTOR_MAX_CLOSED_LOOP_ERROR_TICKS = (int) (MAXIMUM_MOTION_ERROR_INCHES * DRIVE_MOTOR_NATIVE_TICKS_PER_REV / WHEEL_CIRCUMFERENCE_INCHES);
	
	public static final double DRIVE_MOTOR_NATIVE_TICKS_PER_FRAME_DEGREES = DRIVE_MOTOR_NATIVE_TICKS_PER_REV * WHEEL_ROTATION_PER_FRAME_DEGREES;
	
	public static final double DRIVE_MOTOR_OPEN_LOOP_RAMP_SEC   = 0.250;	// Second from neutral to full (easy on the gears)
	public static final double DRIVE_MOTOR_CLOSED_LOOP_RAMP_SEC = 0.0;	    // No ramp rate on closed loop (use Motion Magic)
	
	// PIDF Constants for the DriveSubsystem are empirically derived using
	// the techniques in the TalonSRX Software Reference Manual (2018 Section 12.6)
	//
	// To keep names as short as possible, the fact that all speeds are computed
	// over a 100 ms interval (i.e., encoder changes at 10 Hz) is NOT contained
	// within the names of the constants.
	public final static double DRIVE_MOTOR_FULL_THROTTLE_AVERAGE_SPEED_NATIVE_TICKS = 9926.8;	// per 100 ms, average of 10 samples
	
	// An estimate of the maximum no-load speed is as follows
	public final static double DRIVE_MAXIMUM_NO_LOAD_SPEED_IN_PER_SEC = WHEEL_CIRCUMFERENCE_INCHES * 
																		(DRIVE_MOTOR_FULL_THROTTLE_AVERAGE_SPEED_NATIVE_TICKS /
																		 DRIVE_MOTOR_NATIVE_TICKS_PER_REV) * 10;
	public final static double DRIVE_MAXIMUM_NO_LOAD_SPEED_FT_PER_SEC = DRIVE_MAXIMUM_NO_LOAD_SPEED_IN_PER_SEC / 12.0;
	
	// These Motion Magic values defined the shape of the trapezoidal profile for speed
	// The cruise speed is the maximum speed during the profile and is chosen to keep
	// below the maximum (which varies with battery voltage). The acceleration is the
	// slope allowed to reach the cruise speed or zero (hence, a trapezoid).
	//
	// Setting this to 80% of maximum is a reasonable place to start;
	// However, the acceleration is currently default to reach cruising speed within 1 second 
	// and may need to be increased or decreased depending on static friction limits of tires
	//
	/// TODO: Consider a dashboard selector for floor type to set motion acceleration to match
	/// conditions (e.g., carpet vs wood vs smooth or rough concrete)
	public final static int DRIVE_MOTOR_MOTION_CRUISE_SPEED_NATIVE_TICKS = (int)(0.80 * 
			                                                               DRIVE_MOTOR_FULL_THROTTLE_AVERAGE_SPEED_NATIVE_TICKS);
	public final static int DRIVE_MOTOR_MOTION_ACCELERATION_NATIVE_TICKS = 4346; // 0.26 g on wood//DRIVE_MOTOR_MOTION_CRUISE_SPEED_NATIVE_TICKS;
	
	// The magic number 1023 is in the SRM based on the characteristics of the TalonSRX
	// It is likely based on the internal workings of the A-to-D conversions, but the details
	// are not important at this point; just consider it a scaling factor to make the numbers
	// work for the specific controllers we have.
	public static double driveMotorKf = 1023.0 / DRIVE_MOTOR_FULL_THROTTLE_AVERAGE_SPEED_NATIVE_TICKS; 
	public static double driveMotorKp = 0.6704;				// Is currently 32 x (10% of 1023/error_at_10_rotations)
	public static double driveMotorKi = 0.0;            	// Very small values will help remove any final friction errors
	public static double driveMotorKd = 10 * driveMotorKf;	// Start with 10 x Kp for increased damping of overshoot
	public static int    driveMotorIZone = 0;               // Read up on what this does
	
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
	

	public static final double INTAKE_THROAT_SPEED_PERCENT  =  0.5;
	public static final double OUTTAKE_THROAT_SPEED_PERCENT = -0.5;	/// TODO: May need to be higher to throw longer
	
	
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
	
	public static final double MOTION_PROFILE_PERIOD_MS = 50;
	public static final double MINIMUM_MOVE_FORWARD_INCH = 85;	/// TODO: Check This, front of bumper well across line
	

}
