package org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.Deadzone;
import org.usfirst.frc.team4183.robot.subsystems.BitBucketsSubsystem;

import org.usfirst.frc.team4183.robot.subsystems.SubsystemUtilities.DiagnosticsState;
import org.usfirst.frc.team4183.robot.subsystems.SubsystemUtilities.SubsystemTelemetryState;


public class DriveSubsystem extends BitBucketsSubsystem
{
	private final double INCH_PER_WHEEL_ROT = RobotMap.WHEEL_CIRCUMFERENCE_INCHES;
	

	// Can adjust these to help the robot drive straight with zero turn stick.
	// +Values will add +yaw correct (CCW viewed from top) when going forward.
	private final double YAW_CORRECT_VELOCITY = 0.0;  // Multiplied by inch/sec so value will be small!
	private final double YAW_CORRECT_ACCEL = 0.0;
	
	private final double LOW_SENS_GAIN = 0.6;		
	private final double ALIGN_LOOP_GAIN = 0.04;

	// The counts-per-rev is printed on the encoder -
	// it's the 1st number after the "E4P" or "E4T"
	private final int ENCODER_PULSES_PER_REV = 248; 
	private final boolean REVERSE_SENSOR = false;  
	private final int EDGES_PER_ENCODER_COUNT = 4;
	private double yawSetPoint;
		
	private final WPI_TalonSRX leftFrontMotor;		// User follower mode
	private final WPI_TalonSRX leftRearMotor;

	private final WPI_TalonSRX rightFrontMotor;		// Use follower mode
	private final WPI_TalonSRX rightRearMotor;

	private final DifferentialDrive drive;
	
	private static SendableChooser<SubsystemTelemetryState> telemetryState;
	
    public DriveSubsystem()
    {
    		setName("DriveSubsystem");
    	
    		
    		DIAG_LOOPS_RUN = 10;
    		
	    	leftFrontMotor = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_FRONT_ID);
	    	leftRearMotor = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_REAR_ID);
	    	
	    	/// TODO: Create setupMasterMotor function
	    	/// TODO: Create setupSlaveMotor function
	    	/// Each function should take a list of argument constants for inversion, sense, sensor type, deadbands, etc
	    	
	    	leftFrontMotor.setInverted(RobotMap.LEFT_DRIVE_MOTOR_INVERSION_FLAG);
	    	leftRearMotor.setInverted(RobotMap.LEFT_DRIVE_MOTOR_INVERSION_FLAG);
	    	
	    	leftFrontMotor.setSensorPhase(RobotMap.LEFT_DRIVE_MOTOR_SENSOR_PHASE);
	    	
			// Set relevant frame periods to be at least as fast as periodic rate
	    	// NOTE: This increases load on CAN bus, so pay attention as more motor
	    	// controllers are added to the system
	    	leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 
	    			                            RobotMap.HIGH_STATUS_FRAME_PERIOD_MS, 
	    			                            RobotMap.CONTROLLER_TIMEOUT_MS);
	    	leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 
	    			                            RobotMap.HIGH_STATUS_FRAME_PERIOD_MS, 
	    			                            RobotMap.CONTROLLER_TIMEOUT_MS);
	    	
	    	leftFrontMotor.configNeutralDeadband(RobotMap.LEFT_DRIVE_MOTOR_NEUTRAL_DEADBAND,
	    			                             RobotMap.CONTROLLER_TIMEOUT_MS);
	    	leftRearMotor.configNeutralDeadband(RobotMap.LEFT_DRIVE_MOTOR_NEUTRAL_DEADBAND, 
	    			                            RobotMap.CONTROLLER_TIMEOUT_MS);
	    	
	    	// Always configure peak and nominal outputs to be full scale and 0 respectively
	    	// We will apply limits in other ways, as needed
	    	leftFrontMotor.configPeakOutputForward(1.0, RobotMap.CONTROLLER_TIMEOUT_MS);
		    leftFrontMotor.configPeakOutputReverse(-1.0, RobotMap.CONTROLLER_TIMEOUT_MS);
			leftFrontMotor.configNominalOutputForward(0, RobotMap.CONTROLLER_TIMEOUT_MS);
			leftFrontMotor.configNominalOutputReverse(0, RobotMap.CONTROLLER_TIMEOUT_MS);
			
			leftRearMotor.configPeakOutputForward(1.0, RobotMap.CONTROLLER_TIMEOUT_MS);
			leftRearMotor.configPeakOutputReverse(-1.0, RobotMap.CONTROLLER_TIMEOUT_MS);
			leftRearMotor.configNominalOutputForward(0, RobotMap.CONTROLLER_TIMEOUT_MS);
			leftRearMotor.configNominalOutputReverse(0, RobotMap.CONTROLLER_TIMEOUT_MS);
			
			// Configure for closed loop control
			// Our drives use the "front" motor in a group for control; i.e., where the sensor is located
			leftFrontMotor.configSelectedFeedbackSensor(RobotMap.DRIVE_MOTOR_FEEDBACK_DEVICE, 
					                                    RobotMap.PRIMARY_PID_LOOP, 
					                                    RobotMap.CONTROLLER_TIMEOUT_MS);
			
			// Set closed loop gains in slot0 - see documentation (2018 SRM Section 12.6)
			// The gains are determined empirically following the Software Reference Manual
			// Summary:
			//	Run drive side at full speed, no-load, forward and initiate SelfTest on System Configuration web page
			//  Observe the number of encoder ticks per 100 ms, the % output, and voltage
			//  Collect data in both forward and backwards (e.g., 5 fwd, 5 back)
			//  Average the absolute value of that number, adjust as measured_ticks / percentage_factor
			//  Compute Kf = 1023 / adjusted_tick_average
			//  The using that value, run the Motion Magic forward 10 revolutions at the encoder scale
			//  Note the error (in ticks)
			//  Compute Kp = 0.1 * 1023 / error as a starting point
			//  Command any position through Motion Magic and attempt to turn the motor by hand while holding the command
			//  If the axle turns, keep doubling the Kp until it stops turning (or at leasts resists vigorously without
			//  oscillation); if it oscillates, you must drop the gain.
			//  Run the Motion Magic for at least 10 rotations in each direction
			//  Make not of any misses or overshoot.
			//  If there is unacceptable overshoot then set Kd = 10 * Kp as a starting point and re-test
			//
			//  Put drive train on ground with weight and re-test to see if position is as commanded.
			//  If not, then add SMALL amounts of I-zone and Ki until final error is removed.
			leftFrontMotor.selectProfileSlot(0, RobotMap.PRIMARY_PID_LOOP);
			leftFrontMotor.config_kF(0, 0.1030543579, RobotMap.CONTROLLER_TIMEOUT_MS);		/// TODO: Move constants to map/profile
			leftFrontMotor.config_kP(0, 0.02095*2*2*2*2*2, RobotMap.CONTROLLER_TIMEOUT_MS);
			leftFrontMotor.config_kI(0, 0.0, RobotMap.CONTROLLER_TIMEOUT_MS);
			leftFrontMotor.config_kD(0, 0.0, RobotMap.CONTROLLER_TIMEOUT_MS);
			
			/* set acceleration and vcruise velocity - see documentation */
			leftFrontMotor.configMotionCruiseVelocity(7445, RobotMap.CONTROLLER_TIMEOUT_MS);
			leftFrontMotor.configMotionAcceleration(7445, RobotMap.CONTROLLER_TIMEOUT_MS);
			
			/* zero the sensor */
			leftFrontMotor.setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_LOOP, RobotMap.CONTROLLER_TIMEOUT_MS);
			
	    		    	
	    	// Use follower mode to minimize shearing commands that could occur if
	    	// separate commands are sent to each motor in a group
	    	leftRearMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());

	    	/// TODO: We can bypass this feature (and the effect it can have when the DS
	    	/// or other commands take longer than 100 ms) by using the TalonSRX class
	    	/// rather than the WPI_* variants. Besides, it does not appear that disabling
	    	/// the safety here has any real effect as the "not updating fast enough" message
	    	/// still appears from time to time.
	    	leftFrontMotor.setSafetyEnabled(false);
	    	leftRearMotor.setSafetyEnabled(false);
	    	
	    	rightFrontMotor  = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_FRONT_ID);
	    	rightRearMotor   = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_REAR_ID);
	    	rightFrontMotor.setInverted(RobotMap.RIGHT_DRIVE_MOTOR_INVERSION_FLAG);
	    	rightRearMotor.setInverted(RobotMap.RIGHT_DRIVE_MOTOR_INVERSION_FLAG);

	    	rightFrontMotor.setSensorPhase(RobotMap.RIGHT_DRIVE_MOTOR_SENSOR_PHASE);

	    	rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 
                                                 RobotMap.HIGH_STATUS_FRAME_PERIOD_MS, 
                                                 RobotMap.CONTROLLER_TIMEOUT_MS);
	    	rightFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 
                                                 RobotMap.HIGH_STATUS_FRAME_PERIOD_MS, 
                                                 RobotMap.CONTROLLER_TIMEOUT_MS);
	    	
	    	rightFrontMotor.configNeutralDeadband(RobotMap.RIGHT_DRIVE_MOTOR_NEUTRAL_DEADBAND, 
	    			                              RobotMap.CONTROLLER_TIMEOUT_MS);
	    	rightRearMotor.configNeutralDeadband(RobotMap.RIGHT_DRIVE_MOTOR_NEUTRAL_DEADBAND, 
	    			                             RobotMap.CONTROLLER_TIMEOUT_MS);
	    	
	    	// Always configure peak and nominal outputs to be full scale and 0 respectively
	    	// We will apply limits in other ways, as needed	    	
	    	rightFrontMotor.configPeakOutputForward(1.0, RobotMap.CONTROLLER_TIMEOUT_MS);
			rightFrontMotor.configPeakOutputReverse(-1.0, RobotMap.CONTROLLER_TIMEOUT_MS);
			rightFrontMotor.configNominalOutputForward(0, RobotMap.CONTROLLER_TIMEOUT_MS);
			rightFrontMotor.configNominalOutputReverse(0, RobotMap.CONTROLLER_TIMEOUT_MS);
			
			rightRearMotor.configPeakOutputForward(1.0, RobotMap.CONTROLLER_TIMEOUT_MS);
			rightRearMotor.configPeakOutputReverse(-1.0, RobotMap.CONTROLLER_TIMEOUT_MS);
			rightRearMotor.configNominalOutputForward(0, RobotMap.CONTROLLER_TIMEOUT_MS);
			rightRearMotor.configNominalOutputReverse(0, RobotMap.CONTROLLER_TIMEOUT_MS);
	
			// Configure for closed loop control
			// Our drives use the "front" motor in a group for control; i.e., where the sensor is located
			leftFrontMotor.configSelectedFeedbackSensor(RobotMap.DRIVE_MOTOR_FEEDBACK_DEVICE, 
					                                    RobotMap.PRIMARY_PID_LOOP, 
					                                    RobotMap.CONTROLLER_TIMEOUT_MS);
			
			// Set closed loop gains in slot0 - see documentation (2018 SRM Section 12.6)
			// The gains are determined empirically following the Software Reference Manual
			// Summary:
			//	Run drive side at full speed, no-load, forward and initiate SelfTest on System Configuration web page
			//  Observe the number of encoder ticks per 100 ms, the % output, and voltage
			//  Collect data in both forward and backwards (e.g., 5 fwd, 5 back)
			//  Average the absolute value of that number, adjust as measured_ticks / percentage_factor
			//  Compute Kf = 1023 / adjusted_tick_average
			//  The using that value, run the Motion Magic forward 10 revolutions at the encoder scale
			//  Note the error (in ticks)
			//  Compute Kp = 0.1 * 1023 / error as a starting point
			//  Command any position through Motion Magic and attempt to turn the motor by hand while holding the command
			//  If the axle turns, keep doubling the Kp until it stops turning (or at leasts resists vigorously without
			//  oscillation); if it oscillates, you must drop the gain.
			//  Run the Motion Magic for at least 10 rotations in each direction
			//  Make not of any misses or overshoot.
			//  If there is unacceptable overshoot then set Kd = 10 * Kp as a starting point and re-test
			//
			//  Put drive train on ground with weight and re-test to see if position is as commanded.
			//  If not, then add SMALL amounts of I-zone and Ki until final error is removed.
			rightFrontMotor.selectProfileSlot(0, RobotMap.PRIMARY_PID_LOOP);
			rightFrontMotor.config_kF(0, 0.1030543579, RobotMap.CONTROLLER_TIMEOUT_MS);		/// TODO: Move constants to map/profile
			rightFrontMotor.config_kP(0, 0.02095*2*2*2*2*2, RobotMap.CONTROLLER_TIMEOUT_MS);
			rightFrontMotor.config_kI(0, 0.0, RobotMap.CONTROLLER_TIMEOUT_MS);
			rightFrontMotor.config_kD(0, 0.0, RobotMap.CONTROLLER_TIMEOUT_MS);
			
			/* set acceleration and vcruise velocity - see documentation */
			rightFrontMotor.configMotionCruiseVelocity(7445, RobotMap.CONTROLLER_TIMEOUT_MS);
			rightFrontMotor.configMotionAcceleration(7445, RobotMap.CONTROLLER_TIMEOUT_MS);
			
			/* zero the sensor */
			rightFrontMotor.setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_LOOP, RobotMap.CONTROLLER_TIMEOUT_MS);
			
	    	// Use follower mode to minimize shearing commands that could occur if
	    	// separate commands are sent to each motor in a group
	    	rightRearMotor.set(ControlMode.Follower, rightFrontMotor.getDeviceID());
	    	
	    	rightFrontMotor.setSafetyEnabled(false);
	    	rightRearMotor.setSafetyEnabled(false);
	
	    	// The differential drive simply requires a left and right speed controller
	    	// In this case we can use a single motor controller type on each side
	    	// rather than a SpeedControllerGroup because we set up the follower mode
	    	// and reinforce the follower in the subsystem periodic function
	    	// NOTE: We will want to test that this really works, but it should prevent
	    	// the loop in the SpeedControllerGroup from being sheared by preemption on
	    	// a side that could cause the motors to have different commands on rapid
	    	// reverse boundaries (which is hard on the gears, even for 20 ms).
	    	// The left vs right can still be sheared by preemption but that is generally 
	    	// less harmful on the entire robot as forces and slippage will be absorbed
	    	// through the tires and frame (not JUST the gearbox)
	    	drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
    	
	    	// Now get the other modes set up
	    	setNeutral(NeutralMode.Brake);
	    	telemetryState = new SendableChooser<SubsystemTelemetryState>();
	    	
	    	telemetryState.addDefault("Off", SubsystemTelemetryState.OFF);
	    	telemetryState.addObject( "On",  SubsystemTelemetryState.ON);
	    	
	    	SmartDashboard.putData("DriveTelemetry", telemetryState);
    }
    private double shapeAxis( double x) {
		x = Deadzone.f( x, .05);
		return Math.signum(x) * (x*x);
	}
	
	// +turnStick produces right turn (CW from above, -yaw angle)
    /// TODO: Consider re-designing this to reduce turn by up to 50% at full forward speed
	public void arcadeDrive(double fwdStick, double turnStick) {
		
		if(Robot.oi.btnLowSensitiveDrive.get()) {
			fwdStick *= LOW_SENS_GAIN;
			turnStick *= LOW_SENS_GAIN;
		}
		if(Robot.oi.btnInvertAxis.get()) {
			fwdStick *= -1.0;
		}
		
		// Shape axis for human control
		/// TODO: axis shaping should be controllable via dashboard
		/// see examples of selector for linear, square, cube, and sine
		/// TODO: May want different shapes on fwd and turn
		fwdStick = shapeAxis(fwdStick);
		turnStick = -shapeAxis(turnStick); /// TODO: The extra minus sign here could cause problems
		
		/// TODO: Probably harmless. It is not clear that this 0,0 check will actually
		/// do anything unless shapeAxis actually forces zero for some
		/// shapes. In general, if the value is below the neutral deadband, nothing will move
		/// so the minimum of both left and right deadbands is the determining factor
		if( fwdStick == 0.0 && turnStick == 0.0) {
			setAllMotorsZero();
		}
		else {
			// Turn stick is + to the right;
			// but arcadeDrive 2nd arg + produces left turn
			// (this is +yaw when yaw is defined according to right-hand-rule
			// with z-axis up, so arguably correct).
			// Anyhow need the - sign on turnStick to make it turn correctly.
			drive.arcadeDrive( fwdStick, turnStick + yawCorrect(), false);
		}
	}
	public void doAutoTurn( double turn) {
		drive.arcadeDrive( 0.0, turn, false);				
	}
	
	public void setAlignDrive(boolean start) {
		if(start) {
			yawSetPoint = Robot.imu.getYawDeg();
		} 
	}
	
	public void doAlignDrive(double fwdStick, double turnStick) {
					
		if(Robot.oi.btnLowSensitiveDrive.get())
			fwdStick *= LOW_SENS_GAIN;
		
		if(Robot.oi.btnInvertAxis.get()) {
			fwdStick *= -1.0;
		}
		
		fwdStick = shapeAxis(fwdStick);
		turnStick = shapeAxis(turnStick);
					
		if( fwdStick == 0.0 && turnStick == 0.0) {
			setAllMotorsZero();
		}
		else {
			
			// Turn stick is + to the right,
			// +yaw is CCW looking down,
			// so + stick should lower the setpoint. 
			yawSetPoint += -0.3 * turnStick;
			
			double error = -ALIGN_LOOP_GAIN * (yawSetPoint - Robot.imu.getYawDeg());
			error = -ALIGN_LOOP_GAIN * -Robot.imu.getYawRateDps();
			SmartDashboard.putNumber("IMU_ERROR", error);
			drive.arcadeDrive( fwdStick, error + yawCorrect(), false);
		}
	}
	
	// Autonomous: drive in straight line
	public void doAutoStraight( double fwd) {
		if( fwd == 0.0)
			setAllMotorsZero();
		else {
			double error = ALIGN_LOOP_GAIN * (yawSetPoint - Robot.imu.getYawDeg());				
			drive.arcadeDrive( fwd, error + yawCorrect(), false);				
		}			
	}
	@Override
	protected void initDefaultCommand() 
	{
		setDefaultCommand(new Idle());		
		
	}

	public void disable() {
		setAllMotorsZero();
	}
	
	// Might need to change from .set(value) to .set(mode, value)
	private void setAllMotorsZero() 
	{
		leftFrontMotor.set(ControlMode.PercentOutput, 0.0);
		leftRearMotor.set(ControlMode.PercentOutput, 0.0);
		rightFrontMotor.set(ControlMode.PercentOutput, 0.0);
		rightRearMotor.set(ControlMode.PercentOutput, 0.0);			
	}
	private void setupClosedLoopMaster( WPI_TalonSRX m) 
	{
		// TODO: New functions provide ErrorCode feedback if there is a problem setting up the controller
		
		m.set(ControlMode.Position, 0.0);
		
		m.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, RobotMap.CONTROLLER_TIMEOUT_MS);
		
		// NOTE: The encoder codes per revolution interface no longer exists
		// All of the interfaces operate in native units which are 4x the counts per revolution
		// An encoder that returns 250 counts per rev will be 1000 native units per rev
		// But the resolution is still 360/250 degrees
		// An encoder that return 1024 counts per rev will be 4096 native units per rev
		// But the resolution is still 360/1024 degrees.
		// Basically, we just need to do the math ourselves
		
		//m.setInverted(true);  // TODO: When do we turn this off?
		m.setSelectedSensorPosition(0, 0, RobotMap.CONTROLLER_TIMEOUT_MS);	// Zero the sensor where we are right now
		
		// NOTE: PIDF constants should be determined based on native units
		m.config_kP(0, 0.016, RobotMap.CONTROLLER_TIMEOUT_MS); // May be able to increase gain a bit	
		m.config_kI(0, 0, RobotMap.CONTROLLER_TIMEOUT_MS);
		m.config_kD(0, 0, RobotMap.CONTROLLER_TIMEOUT_MS); 
		m.config_kF(0, 0, RobotMap.CONTROLLER_TIMEOUT_MS);
		m.config_IntegralZone(0, 0, RobotMap.CONTROLLER_TIMEOUT_MS);
		
		m.configClosedloopRamp(0.250, RobotMap.CONTROLLER_TIMEOUT_MS); // Smoothes things a bit: Don't switch from neutral to full too quickly
		
		// TODO: Need to understand the implication of this error limit
		// If it is in "ticks" or "pulse" or whatever, then how big are 8 ticks
		// E.g., if encoder is 256 steps per revolution then 8/256 is 11.25 degress, which is actually
		// quite large. So we need to figure this out if we want to have real control.
		m.configAllowableClosedloopError(0, 0, RobotMap.CONTROLLER_TIMEOUT_MS);  // Specified in native "ticks"?
		
		m.configPeakOutputForward(1.0, RobotMap.CONTROLLER_TIMEOUT_MS);
		m.configPeakOutputReverse(-1.0, RobotMap.CONTROLLER_TIMEOUT_MS);
		m.configNominalOutputForward(1.0/3.0, RobotMap.CONTROLLER_TIMEOUT_MS);
		m.configNominalOutputReverse(-1.0/3.0, RobotMap.CONTROLLER_TIMEOUT_MS);
					
	}
	
	public void doLockDrive(double value) 
	{
		leftFrontMotor.set(ControlMode.Position, value);
		leftRearMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());	// Reinforce
		rightFrontMotor.set(ControlMode.Position, value);
		rightRearMotor.set(ControlMode.Follower, rightFrontMotor.getDeviceID());			
	}
	public void setLockDrive( boolean start) 
	{

		if( start) 
		{
			setupClosedLoopMaster(leftFrontMotor);
			setupClosedLoopMaster(rightFrontMotor);

			leftRearMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());	// Reinforce
			leftRearMotor.setInverted(false); // Follow the front
			rightRearMotor.set(ControlMode.Follower, rightFrontMotor.getDeviceID());			
			rightRearMotor.setInverted(false); // Follow the front
		}
		else 
		{
			leftFrontMotor.set(ControlMode.PercentOutput,0.0);
			leftRearMotor.set(ControlMode.PercentOutput,0.0);
			rightFrontMotor.set(ControlMode.PercentOutput,0.0);
			rightRearMotor.set(ControlMode.PercentOutput,0.0);							
		}
	}
	

	/** 
	 * setNeutral is a pass through interface to each motor in the subsystem
	 * 
	 * @param neutralMode is either Coast or Brake. Braking will apply force to come to a stop at zero input
	 */
	private void setNeutral(NeutralMode neutralMode) 
	{	
		leftFrontMotor.setNeutralMode(neutralMode);
		leftRearMotor.setNeutralMode(neutralMode);
		rightFrontMotor.setNeutralMode(neutralMode);
		rightRearMotor.setNeutralMode(neutralMode);
		
	}
	private double yawCorrect() {
		return YAW_CORRECT_VELOCITY * getFwdVelocity_ips() 
				+ YAW_CORRECT_ACCEL * getFwdCurrent();
	}
	public double getRightPosition_inch() {
		// Right motor encoder reads -position when going forward!
		// TODO: This is wrong! Need new constants
		return -INCH_PER_WHEEL_ROT * rightFrontMotor.getSelectedSensorPosition(RobotMap.PRIMARY_PID_LOOP);						
	}
	
	private int getMotorNativeUnits(WPI_TalonSRX m) {
		return m.getSelectedSensorPosition(RobotMap.PRIMARY_PID_LOOP);
	}
	
	public int getRightNativeUnits() {
		return getMotorNativeUnits(rightFrontMotor);
	}
	
	public int getLeftNativeUnits() {
		return getMotorNativeUnits(leftFrontMotor);
	}
	
	private double getMotorEncoderUnits(WPI_TalonSRX m) {
		return getMotorNativeUnits(m)/EDGES_PER_ENCODER_COUNT;
	}
	
	public double getRightEncoderUnits() {
		return getMotorEncoderUnits(rightFrontMotor);
	}
	
	public double getLeftEncoderUnits() {
		return getMotorEncoderUnits(leftFrontMotor);
	}
	
	private ControlMode getMotorMode(WPI_TalonSRX m) {
		return m.getControlMode();
	}
	
	public ControlMode getRightFrontMode() {
		return getMotorMode(rightFrontMotor);
	}
	
	public ControlMode getLeftFrontMode() {
		return getMotorMode(leftFrontMotor);
	}
	
	public ControlMode getLeftRearMode() {
		return getMotorMode(leftRearMotor);
	}
	
	public ControlMode getRightRearMode() {
		return getMotorMode(rightRearMotor);
	}
	
	public double inchesToEncoderTicks(double inches) {
		return ENCODER_PULSES_PER_REV * EDGES_PER_ENCODER_COUNT * inches / RobotMap.WHEEL_CIRCUMFERENCE_INCHES;
	}

	public double getFwdVelocity_ips() {
		// Right side motor reads -velocity when going forward!
		double fwdSpeedRpm = (leftFrontMotor.getSelectedSensorVelocity(RobotMap.PRIMARY_PID_LOOP) - rightFrontMotor.getSelectedSensorVelocity(RobotMap.PRIMARY_PID_LOOP))/2.0;
		return (INCH_PER_WHEEL_ROT / 60.0) * fwdSpeedRpm;
	}
	public double getFwdCurrent() {
		// OutputCurrent always positive so apply sign of drive voltage to get real answer.
		// Also, right side has -drive when going forward!
		double leftFront = leftFrontMotor.getOutputCurrent() * Math.signum( leftFrontMotor.getMotorOutputVoltage());
		double leftRear = leftRearMotor.getOutputCurrent() * Math.signum( leftRearMotor.getMotorOutputVoltage());
		double rightFront = -rightFrontMotor.getOutputCurrent() * Math.signum( rightFrontMotor.getMotorOutputVoltage());
		double rightRear = -rightRearMotor.getOutputCurrent() * Math.signum( rightRearMotor.getMotorOutputVoltage());
		return (leftFront + leftRear + rightFront + rightRear)/4.0;
	}
	
	public double getPosition_inch() {
		// TODO Auto-generated method stub
		return 0;
	}
	
	private void setupPositionControl(WPI_TalonSRX m) {
		m.setSelectedSensorPosition(0, 0, RobotMap.CONTROLLER_TIMEOUT_MS);
		m.config_kP(0, SmartDashboard.getNumber("Kp", 0.016), RobotMap.CONTROLLER_TIMEOUT_MS); // May be able to increase gain a bit	
		m.config_kI(0, SmartDashboard.getNumber("Ki", 0.016), RobotMap.CONTROLLER_TIMEOUT_MS);
		m.config_kD(0, SmartDashboard.getNumber("Kd", 0.016), RobotMap.CONTROLLER_TIMEOUT_MS);
		
		m.set(ControlMode.PercentOutput, 0.0);
	}
	
	public void setupPositionControl() {
		setupPositionControl(leftFrontMotor);
		setupPositionControl(leftRearMotor);
		setupPositionControl(rightFrontMotor);
		setupPositionControl(rightRearMotor);
	}
	
	private void setPos(WPI_TalonSRX m, double value) {
		
		m.set(ControlMode.Position, value);
	}
	
	public void setPos(double value) {
		setPos(leftFrontMotor, inchesToEncoderTicks(value));
		setPos(rightFrontMotor, -inchesToEncoderTicks(value));
		setPos(leftRearMotor, inchesToEncoderTicks(value));
		setPos(rightRearMotor, -inchesToEncoderTicks(value));
	}
	
	/* Any hardware devices used in this subsystem must
	*  have a check here to see if it is still connected and 
	*  working properly. For motors check for current draw.
	*  Return true iff all devices are working properly. Otherwise
	*  return false. This sets all motors to percent output
	*/
	@Override
	public void diagnosticsInit() {
		
	}
	
	@Override
	public void diagnosticsExecute() {

		/* Init Diagnostics */
		SmartDashboard.putBoolean("RunningDiag", true);
		
		rightFrontMotor.set(ControlMode.PercentOutput, RobotMap.MOTOR_TEST_PERCENT);
		rightRearMotor.set(ControlMode.PercentOutput, -RobotMap.MOTOR_TEST_PERCENT);
		leftFrontMotor.set(ControlMode.PercentOutput, -RobotMap.MOTOR_TEST_PERCENT);
		leftRearMotor.set(ControlMode.PercentOutput, RobotMap.MOTOR_TEST_PERCENT);
	}
	
	@Override
	public void diagnosticsCheck() {
		/* Reset flag */
		runDiagnostics = false;
		
		/* Diagnostics */
		lastKnownState = DiagnosticsState.PASS;
		SmartDashboard.putBoolean(getName() + "Diagnostics", true); // All good until we find a fault
		
		SmartDashboard.putBoolean("DiagnosticsFR", true);
		if(rightFrontMotor.getOutputCurrent() <= RobotMap.MINUMUM_MOTOR_CURR) {
			SmartDashboard.putBoolean("DiagnosticsFR", false);
			SmartDashboard.putBoolean(getName() + "Diagnostics", false);
			lastKnownState = DiagnosticsState.FAIL;
		}
		rightFrontMotor.set(ControlMode.PercentOutput, 0.0);
		
		SmartDashboard.putBoolean("DiagnosticsBR", true);
		if(rightRearMotor.getOutputCurrent() <= RobotMap.MINUMUM_MOTOR_CURR) {
			SmartDashboard.putBoolean("DiagnosticsBR", false);
			SmartDashboard.putBoolean(getName() + "Diagnostics", false);
			lastKnownState = DiagnosticsState.FAIL;
		}
		rightRearMotor.set(ControlMode.PercentOutput, 0.0);
		
		SmartDashboard.putBoolean("DiagnosticsFL", true);
		if(leftFrontMotor.getOutputCurrent() <= RobotMap.MINUMUM_MOTOR_CURR) {
			SmartDashboard.putBoolean("DiagnosticsFL", false);
			SmartDashboard.putBoolean(getName() + "Diagnostics", false);
			lastKnownState = DiagnosticsState.FAIL;
		}
		leftFrontMotor.set(ControlMode.PercentOutput, 0.0);
		
		SmartDashboard.putBoolean("DiagnosticsBL", true);
		if(leftRearMotor.getOutputCurrent() <= RobotMap.MINUMUM_MOTOR_CURR) {
			SmartDashboard.putBoolean("DiagnosticsBL", false);
			SmartDashboard.putBoolean(getName() + "Diagnostics", false);
			lastKnownState = DiagnosticsState.FAIL;
		}
		leftRearMotor.set(ControlMode.PercentOutput, 0.0);
	}
	
	@Override
	public void setDiagnosticsFlag(boolean state) {
		runDiagnostics = state;
	}
	
	@Override
	public boolean getDiagnosticsFlag() {
		return runDiagnostics;
	}
	
	@Override
	public void periodic() {
		
		// Always make the extra motors in the system stay in the follower
		// mode; this simplifies the timing in any motor group by not needing
		// to send the command to more than one motor per side
		leftRearMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());
		rightRearMotor.set(ControlMode.Follower, rightFrontMotor.getDeviceID());
		
		if(telemetryState.getSelected() == SubsystemTelemetryState.ON) {
//			SmartDashboard.putNumber("ReadMotorCurrent", 
//					rightRearMotor.getOutputCurrent());
			SmartDashboard.putNumber("Yaw_Deg",
					Robot.imu.getYawDeg());
			SmartDashboard.putNumber("Yaw_Rate", 
					Robot.imu.getYawRateDps());
			
			SmartDashboard.putNumber( "RightNativeUnits", 
					getRightNativeUnits());
			SmartDashboard.putNumber( "LeftNativeUnits", 
					getLeftNativeUnits());
			SmartDashboard.putNumber( "RightEncoderUnits", 
					getRightEncoderUnits());
			SmartDashboard.putNumber( "LeftEncoderUnits", 
					getLeftEncoderUnits());	
			
		}
		
	}

}

