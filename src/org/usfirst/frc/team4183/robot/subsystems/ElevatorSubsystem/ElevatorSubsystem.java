package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.BitBucketsSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class ElevatorSubsystem extends BitBucketsSubsystem {

	private final WPI_TalonSRX elevatorMotorA;
	
	private final WPI_TalonSRX elevatorMotorB;
	
	private final int UNITS_PER_FEET = 1000;
	
	private final int ENCODER_TICKS_REV = 2048;
	
	//adjust this later for the driver control
	private final int deltaPos = UNITS_PER_FEET;

	public ElevatorSubsystem()
	{
		elevatorMotorA = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR_A_ID);
		elevatorMotorB = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR_B_ID);	
		
		setupClosedLoopMaster(elevatorMotorA);
		elevatorMotorB.set(ControlMode.Follower, RobotMap.ELEVATOR_MOTOR_A_ID);
		
		setAllMotorsZero();
	}
	
	private void setupClosedLoopMaster( WPI_TalonSRX m) 
	{
		// TODO: New functions provide ErrorCode feedback if there is a problem setting up the controller
		
		m.set(ControlMode.Position, 0.0);
		
		//double check this type of encoder
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
	
	public void disable()
	{
		setAllMotorsZero();
	}
	
	public int inchesToTicks(double inches)
	{
		return (int) (inches/RobotMap.INCH_EXTENSION_ROT);
	}
	
	public void addToPosition(double joyStickVal)
	{
		elevatorMotorA.set(ControlMode.Position, elevatorMotorA.getSelectedSensorPosition(RobotMap.PRIMARY_PID_LOOP)+Math.floor(joyStickVal*deltaPos));
		elevatorMotorB.set(ControlMode.Follower, RobotMap.ELEVATOR_MOTOR_A_ID);
	}
	
	public void goToPosition(int ticks)
	{
		elevatorMotorA.set(ControlMode.Position, ticks);
		elevatorMotorB.set(ControlMode.Follower,RobotMap.ELEVATOR_MOTOR_A_ID);
	}
	
	public void setAllMotorsZero()
	{
		elevatorMotorA.set(ControlMode.PercentOutput,0);
		elevatorMotorA.set(ControlMode.Follower,RobotMap.ELEVATOR_MOTOR_A_ID);
	}
	
	public void setSystemPower(double power)
	{
		elevatorMotorA.set(ControlMode.PercentOutput,power);
		if (elevatorMotorB.getControlMode() != ControlMode.Follower)
		{
			elevatorMotorB.set(ControlMode.Follower,RobotMap.ELEVATOR_MOTOR_A_ID);
		}
	}



	@Override
	public void diagnosticsCheck() {
		// TODO Auto-generated method stub
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new Idle());		
	}

	@Override
	public void diagnosticsInit() {
		// TODO Auto-generated method stub
		
	}


	@Override
	public void periodic() {
		// TODO Auto-generated method stub
		
	}
	
	@Override
	public void diagnosticsExecute() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void setDiagnosticsFlag(boolean enable) {
		runDiagnostics = enable;
		
	}

	@Override
	public boolean getDiagnosticsFlag() {
		// TODO Auto-generated method stub
		return runDiagnostics;
	}
	
	
}
