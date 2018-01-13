package org.usfirst.frc.team4183.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.commands.DriveSubsystem.Idle;
import org.usfirst.frc.team4183.utils.Deadzone;

public class DriveSubsystem extends Subsystem
{
	private final double INCH_PER_WHEEL_ROT = RobotMap.INCH_PER_WHEEL_ROT;
	
	private final int CONTROLLER_TIMEOUT_MS = 100; // Default timeout to wait for configuration response

	// Can adjust these to help the robot drive straight with zero turn stick.
	// +Values will add +yaw correct (CCW viewed from top) when going forward.
	private final double YAW_CORRECT_VELOCITY = 0.0;  // Multiplied by inch/sec so value will be small!
	private final double YAW_CORRECT_ACCEL = 0.0;
	
	private final double LOW_SENS_GAIN = 0.6;		
	private final double ALIGN_LOOP_GAIN = 0.04;

	// The counts-per-rev is printed on the encoder -
	// it's the 1st number after the "E4P" or "E4T"
	private final int ENCODER_PULSES_PER_REV = 250; 
	private final boolean REVERSE_SENSOR = false;  
	private double yawSetPoint;
	
		
	private final WPI_TalonSRX leftFrontMotor;		// User follower mode
	private final WPI_TalonSRX leftRearMotor;

	private final WPI_TalonSRX rightFrontMotor;		// Use follower mode
	private final WPI_TalonSRX rightRearMotor;

	private final DifferentialDrive drive;
	
    public DriveSubsystem()
    {
	    	leftFrontMotor = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_FRONT_ID);
	    	leftRearMotor = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_REAR_ID);
	    	
	    	// Use follower mode to minimize shearing commands that could occur if
	    	// separate commands are sent to each motor in a group
	    	leftRearMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());
	    	
	    	rightFrontMotor  = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_FRONT_ID);
	    	rightRearMotor   = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_REAR_ID);
	
	    	// Use follower mode to minimize shearing commands that could occur if
	    	// separate commands are sent to each motor in a group
	    	rightRearMotor.set(ControlMode.Follower, rightFrontMotor.getDeviceID());
	
	    	// The differential drive simply requires a left and right speed controller
	    	// In this case we can use a single motor controller type on each side
	    	// rather than a SpeedControllerGroup because we set up the follower mode
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
    }
    private double shapeAxis( double x) {
		x = Deadzone.f( x, .05);
		return Math.signum(x) * (x*x);
	}
	
	// +turnStick produces right turn (CW from above, -yaw angle)
	public void arcadeDrive(double fwdStick, double turnStick) {
		
		if(Robot.oi.btnLowSensitiveDrive.get()) {
			fwdStick *= LOW_SENS_GAIN;
			turnStick *= LOW_SENS_GAIN;
		}
		if(Robot.oi.btnInvertAxis.get()) {
			fwdStick *= -1.0;
		}
		
		// Shape axis for human control
		fwdStick = shapeAxis(fwdStick);
		turnStick = shapeAxis(turnStick);
					
		if( fwdStick == 0.0 && turnStick == 0.0) {
			setAllMotorsZero();
		}
		else {
			// Turn stick is + to the right;
			// but arcadeDrive 2nd arg + produces left turn
			// (this is +yaw when yaw is defined according to right-hand-rule
			// with z-axis up, so arguably correct).
			// Anyhow need the - sign on turnStick to make it turn correctly.
			drive.arcadeDrive( fwdStick, -turnStick + yawCorrect(), false);
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
			
			double error = ALIGN_LOOP_GAIN * (yawSetPoint - Robot.imu.getYawDeg());				
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
	
	private void setAllMotorsZero() 
	{
		leftFrontMotor.set(0.0);
		leftRearMotor.set(0.0);
		rightFrontMotor.set(0.0);
		rightRearMotor.set(0.0);			
	}
	private void setupClosedLoopMaster( WPI_TalonSRX m) 
	{
		// TODO: New functions provide ErrorCode feedback if there is a problem setting up the controller
		
		m.set(ControlMode.Position,0.0);
		m.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, CONTROLLER_TIMEOUT_MS);
		
		// NOTE: The encoder codes per revolution interface no longer exists
		// All of the interfaces operate in native units which are 4x the counts per revolution
		// An encoder that returns 250 counts per rev will be 1000 native units per rev
		// But the resolution is still 360/250 degrees
		// An encoder that return 1024 counts per rev will be 4096 native units per rev
		// But the resolution is still 360/1024 degrees.
		// Basically, we just need to do the math ourselves
		
		m.setInverted(true);
		m.setSelectedSensorPosition(0, 0, CONTROLLER_TIMEOUT_MS);	// Zero the sensor where we are right now
		
		// NOTE: PIDF constants should be determined based on native units
		m.config_kP(0, 0.4, CONTROLLER_TIMEOUT_MS); // May be able to increase gain a bit	
		m.config_kI(0, 0, CONTROLLER_TIMEOUT_MS);
		m.config_kD(0, 0, CONTROLLER_TIMEOUT_MS); 
		m.config_kF(0, 0, CONTROLLER_TIMEOUT_MS);
		m.config_IntegralZone(0, 0, CONTROLLER_TIMEOUT_MS);
		
		m.configClosedloopRamp(0.250, CONTROLLER_TIMEOUT_MS); // Smoothes things a bit: Don't switch from neutral to full too quickly
		
		// TODO: Need to understand the implication of this error limit
		// If it is in "ticks" or "pulse" or whatever, then how big are 8 ticks
		// E.g., if encoder is 256 steps per revolution then 8/256 is 11.25 degress, which is actually
		// quite large. So we need to figure this out if we want to have real control.
		m.configAllowableClosedloopError(0, 8, CONTROLLER_TIMEOUT_MS);  // Specified in native "ticks"?
		
		m.configPeakOutputForward(1.0, CONTROLLER_TIMEOUT_MS);
		m.configPeakOutputReverse(-1.0, CONTROLLER_TIMEOUT_MS);
		m.configNominalOutputForward(1.0/3.0, CONTROLLER_TIMEOUT_MS);
		m.configNominalOutputReverse(-1.0/3.0, CONTROLLER_TIMEOUT_MS);
					
	}
	
	public void doLockDrive(double value) 
	{
		leftFrontMotor.set(value);
		leftRearMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());	// Reinforce
		rightFrontMotor.set(value);
		rightRearMotor.set(ControlMode.Follower, rightFrontMotor.getDeviceID());			
	}
	public void setLockDrive( boolean start) 
	{

		if( start) 
		{
			setupClosedLoopMaster(leftFrontMotor);
			setupClosedLoopMaster(rightFrontMotor);

			leftRearMotor.set(ControlMode.Follower, leftFrontMotor.getDeviceID());	// Reinforce
			leftRearMotor.setInverted(false);; // Follow the front
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
	


	private void setNeutral(NeutralMode neutralMode) 
	{
//		Do not attempt to stop the motor. Instead allow it to coast to a stop
//		without applying resistance. 
//		Coast(1),
//		Stop the motor's rotation by applying a force. 
//		Brake(2);
		
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
		return -INCH_PER_WHEEL_ROT * rightFrontMotor.getSelectedSensorPosition(ENCODER_PULSES_PER_REV);						
	}

	public double getFwdVelocity_ips() {
		// Right side motor reads -velocity when going forward!
		double fwdSpeedRpm = (leftFrontMotor.getSelectedSensorVelocity(ENCODER_PULSES_PER_REV) - rightFrontMotor.getSelectedSensorVelocity(ENCODER_PULSES_PER_REV))/2.0;
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
}

