package org.usfirst.frc.team4183.robot.subsystems;

import com.ctre.CANTalon;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.commands.DriveSubsystem.Idle;
import org.usfirst.frc.team4183.utils.Deadzone;

public class DriveSubsystem extends Subsystem
{
	private final double INCH_PER_WHEEL_ROT = RobotMap.INCH_PER_WHEEL_ROT;

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
	
		
	private final WPI_TalonSRX leftMotorFront;		// User follower mode
	private final WPI_TalonSRX leftMotorBack;

	private final WPI_TalonSRX rightMotorFront;		// Use follower mode
	private final WPI_TalonSRX rightMotorBack;

	private final DifferentialDrive drive;
	
    public DriveSubsystem()
    {
	    	leftMotorFront = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_FRONT_ID);
	    	leftMotorBack = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_BACK_ID);
	    	
	    	// Use follower mode to minimize shearing commands that could occur if
	    	// separate commands are sent to each motor in a group
	    	leftMotorBack.set(ControlMode.Follower, RobotMap.LEFT_DRIVE_MOTOR_FRONT_ID);
	    	
	    	rightMotorFront  = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_FRONT_ID);
	    	rightMotorBack   = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_FRONT_ID);
	
	    	// Use follower mode to minimize shearing commands that could occur if
	    	// separate commands are sent to each motor in a group
	    	rightMotorBack.set(ControlMode.Follower, RobotMap.RIGHT_DRIVE_MOTOR_FRONT_ID);
	
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
	    	drive = new DifferentialDrive(leftMotorFront, rightMotorFront);
    	
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
		leftMotorFront.set(0.0);
		leftMotorBack.set(0.0);
		rightMotorFront.set(0.0);
		rightMotorBack.set(0.0);			
	}
	private void setupClosedLoopMaster( WPI_TalonSRX m) {
		
		m.set(ControlMode.Position,0.0);
		m.setFeedbackDevice(WPI_TalonSRX.FeedbackDevice.QuadEncoder);
		m.configEncoderCodesPerRev(ENCODER_PULSES_PER_REV);
		m.reverseSensor(REVERSE_SENSOR); 
		m.setPosition(0.0);
		
		m.setPID(0.4, 0.0, 0.0); // Might be able to increase gain a bit
		m.setF(0.0);
		m.setIZone(0);
		m.setCloseLoopRampRate(50.0);    // Smoothes things a bit
		m.setAllowableClosedLoopErr(8);  // Specified in CANTalon "ticks"
		m.configNominalOutputVoltage(+4.0, -4.0);
		m.configPeakOutputVoltage(+12.0, -12.0);			
	}
	
	public void doLockDrive(double value) {
		leftFrontMotor.set(value);
		leftRearMotor.set(leftFrontMotor.getDeviceID());
		rightFrontMotor.set(value);
		rightRearMotor.set(rightFrontMotor.getDeviceID());			
	}
	public void setLockDrive( boolean start) {

		if( start) {
			setupClosedLoopMaster(leftMotorFront);
			setupClosedLoopMaster(rightMotorFront);

			leftRearMotor.changeControlMode(CANTalon.TalonControlMode.Follower);
			leftRearMotor.reverseOutput(false); // Follow the front
			rightRearMotor.changeControlMode(CANTalon.TalonControlMode.Follower);
			rightRearMotor.reverseOutput(false); // Follow the front
		}
		else {
			leftFrontMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
			leftRearMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
			rightFrontMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
			rightRearMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);							
		}
	}
	


	private void setNeutral(NeutralMode neutralMode) 
	{
//		Do not attempt to stop the motor. Instead allow it to coast to a stop
//		without applying resistance. 
//		Coast(1),
//		Stop the motor's rotation by applying a force. 
//		Brake(2);
		
		leftMotorFront.setNeutralMode(neutralMode);
		leftMotorBack.setNeutralMode(neutralMode);
		rightMotorFront.setNeutralMode(neutralMode);
		rightMotorBack.setNeutralMode(neutralMode);
		
	}
	private double yawCorrect() {
		return YAW_CORRECT_VELOCITY * getFwdVelocity_ips() 
				+ YAW_CORRECT_ACCEL * getFwdCurrent();
	}
	public double getRightPosition_inch() {
		// Right motor encoder reads -position when going forward!
		return -INCH_PER_WHEEL_ROT * rightMotorFront.getSelectedSensorPosition(ENCODER_PULSES_PER_REV);						
	}

	public double getFwdVelocity_ips() {
		// Right side motor reads -velocity when going forward!
		double fwdSpeedRpm = (leftMotorFront.getSelectedSensorVelocity(ENCODER_PULSES_PER_REV) - rightMotorFront.getSelectedSensorVelocity(ENCODER_PULSES_PER_REV))/2.0;
		return (INCH_PER_WHEEL_ROT / 60.0) * fwdSpeedRpm;
	}
	public double getFwdCurrent() {
		// OutputCurrent always positive so apply sign of drive voltage to get real answer.
		// Also, right side has -drive when going forward!
		double leftFront = leftMotorFront.getOutputCurrent() * Math.signum( leftMotorFront.getMotorOutputVoltage());
		double leftRear = leftMotorBack.getOutputCurrent() * Math.signum( leftMotorBack.getMotorOutputVoltage());
		double rightFront = -rightMotorFront.getOutputCurrent() * Math.signum( rightMotorFront.getMotorOutputVoltage());
		double rightRear = -rightMotorBack.getOutputCurrent() * Math.signum( rightMotorBack.getMotorOutputVoltage());
		return (leftFront + leftRear + rightFront + rightRear)/4.0;
	}
}

