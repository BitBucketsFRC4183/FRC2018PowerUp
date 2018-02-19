package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.BitBucketsSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;


public class ElevatorSubsystem extends BitBucketsSubsystem {

	private final TalonSRX elevatorMotorA;
	
	private final TalonSRX throatMotorA;
	private final TalonSRX throatMotorB;
	
	private final DoubleSolenoid brakePneu;
	
	private final int UNITS_PER_FEET = 1000;
	
	private final int ENCODER_TICKS_REV = 2048;
	
	static enum ElevatorPositions
	{
		SCALE(10000), SWITCH(1000), TRANS(500), INIT(0), MANUAL(-2), TRIGGER(600);
		
		private int units;
		ElevatorPositions(int units)
		{
			this.units = units;
		}
		
		public int getUnits()
		{
			return this.units;
		}
	}
	
	public static final double timeUntilBrakeSec = .75;
	
	public static int holdUnits = 0;
	
	//adjust this later for the driver control
	private final int deltaPos = UNITS_PER_FEET;
	
	ElevatorPositions currentElevatorPosition = ElevatorPositions.INIT;
	
	//This is used to open the intake mandibles if the position is too close.

	public ElevatorSubsystem()
	{
		elevatorMotorA = new TalonSRX(RobotMap.ELEVATOR_MOTOR_A_ID);
		
		throatMotorA = new TalonSRX(RobotMap.THROAT_MOTOR_A_ID);
		throatMotorA.setInverted(true);
		throatMotorB = new TalonSRX(RobotMap.THROAT_MOTOR_B_ID);
		
		brakePneu = new DoubleSolenoid(RobotMap.ELEVATOR_PNEUMA_BRAKE_CLOSE_CHANNEL,RobotMap.ELEVATOR_PNEUMA_BRAKE_OPEN_CHANNEL);
		
		elevatorMotorA.setSensorPhase(RobotMap.ELEVATOR_MOTOR_SENSOR_PHASE);
		
		elevatorMotorA.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 
                RobotMap.HIGH_STATUS_FRAME_PERIOD_MS, 
                RobotMap.CONTROLLER_TIMEOUT_MS);
		elevatorMotorA.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 
                RobotMap.HIGH_STATUS_FRAME_PERIOD_MS, 
                RobotMap.CONTROLLER_TIMEOUT_MS);
		
		elevatorMotorA.configNeutralDeadband(RobotMap.ELEVATOR_MOTOR_NEUTRAL_DEADBAND, RobotMap.CONTROLLER_TIMEOUT_MS);
		
		elevatorMotorA.configPeakOutputForward(1.0, RobotMap.CONTROLLER_TIMEOUT_MS);
	    elevatorMotorA.configPeakOutputReverse(-1.0, RobotMap.CONTROLLER_TIMEOUT_MS);
		elevatorMotorA.configNominalOutputForward(0, RobotMap.CONTROLLER_TIMEOUT_MS);
		elevatorMotorA.configNominalOutputReverse(0, RobotMap.CONTROLLER_TIMEOUT_MS);
		
		elevatorMotorA.configSelectedFeedbackSensor(RobotMap.ELEVATOR_MOTOR_FEEDBACK_DEVICE, RobotMap.PRIMARY_PID_LOOP, RobotMap.CONTROLLER_TIMEOUT_MS);
		
		//FPID Configurations
		elevatorMotorA.selectProfileSlot(0, RobotMap.PRIMARY_PID_LOOP);
		elevatorMotorA.config_kF(0, RobotMap.elevatorMotorKf, RobotMap.CONTROLLER_TIMEOUT_MS);		/// TODO: Move constants to map/profile
		elevatorMotorA.config_kP(0, RobotMap.elevatorMotorKp, RobotMap.CONTROLLER_TIMEOUT_MS);
		elevatorMotorA.config_kI(0, RobotMap.elevatorMotorKi, RobotMap.CONTROLLER_TIMEOUT_MS);
		elevatorMotorA.config_kD(0, RobotMap.elevatorMotorKd, RobotMap.CONTROLLER_TIMEOUT_MS);
		elevatorMotorA.config_IntegralZone(0, RobotMap.elevatorMotorIZone, RobotMap.CONTROLLER_TIMEOUT_MS);
		
		/* set acceleration and vcruise velocity - see documentation */
		elevatorMotorA.configMotionCruiseVelocity(RobotMap.ELEVATOR_MOTOR_MOTION_CRUISE_SPEED_NATIVE_TICKS, 
				                                  RobotMap.CONTROLLER_TIMEOUT_MS);
		elevatorMotorA.configMotionAcceleration(RobotMap.ELEVATOR_MOTOR_MOTION_ACCELERATION_NATIVE_TICKS, 
				                                RobotMap.CONTROLLER_TIMEOUT_MS);
		
		elevatorMotorA.setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_LOOP, RobotMap.CONTROLLER_TIMEOUT_MS);
		
		setAllMotorsZero();
	}
	
	public void disableThroat()
	{
		throatMotorA.set(ControlMode.PercentOutput,0);
		throatMotorB.set(ControlMode.PercentOutput,0);
	}
	
	public void intakeThroat()
	{
		throatMotorA.set(ControlMode.PercentOutput,.5);
		throatMotorB.set(ControlMode.PercentOutput,.5);
	}
	
	public void outtakeThroat()
	{
		throatMotorA.set(ControlMode.PercentOutput,-.5);
		throatMotorB.set(ControlMode.PercentOutput,-.5);
	}
	
	public void setElevPos(ElevatorPositions elevPos)
	{
		this.currentElevatorPosition = elevPos;
	}
	
	public ElevatorPositions getElevPos()
	{
		return this.currentElevatorPosition;
	}
	
	//method that checks if the intake mandibles should be open
	public boolean posGreaterThanMin()
	{
		if (elevatorMotorA.getSelectedSensorPosition(RobotMap.PRIMARY_PID_LOOP) > ElevatorPositions.TRIGGER.getUnits())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	public boolean closeToDesiredPos()
	{
	return (Math.abs(elevatorMotorA.getSelectedSensorPosition(RobotMap.PRIMARY_PID_LOOP)-currentElevatorPosition.getUnits()) < 50);
	}
	//returns true if the elevator is close to the floor Position
	public boolean posCloseToInit()
	{
		if (Math.abs(elevatorMotorA.getSelectedSensorPosition(RobotMap.PRIMARY_PID_LOOP)-ElevatorPositions.INIT.getUnits()) < 50)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	//holds the current Encoder position MAKE SURE TO SET THIS METHOD TO FALSE AFTER HOLDING POSITION TO RESET HOLDUNITS
	public void holdEncodPos(boolean holdUnitsBol)
	{
		if (holdUnitsBol)
		{
			if (holdUnits == 0)
			{
				holdUnits = elevatorMotorA.getSelectedSensorPosition(RobotMap.PRIMARY_PID_LOOP);
			}
			goToPosition(holdUnits);
		}
		else
		{
		holdUnits = 0;
		}
		}
	public void engageBrake()
	{
		brakePneu.set(DoubleSolenoid.Value.kForward);
	}
	
	public void disengageBrake()
	{
		brakePneu.set(DoubleSolenoid.Value.kReverse);
	}
	
	//return true if a cube is present
	
	public void releasePos()
	{
		disengageBrake();
		holdEncodPos(false);
	}
	
	//this one engages the brake
	public void holdPos()
	{
		holdEncodPos(true);
		engageBrake();
		setAllMotorsZero();
	}
	
	public void disable()
	{
		setAllMotorsZero();
	}
	
	public int inchesToUnits(double inches)
	{
		return (int) (inches/RobotMap.ELEVATOR_INCHES_PER_NATIVE_TICKS);
	}
	public double unitsToInches(int units)
	{
		return (double)(units) * RobotMap.ELEVATOR_INCHES_PER_NATIVE_TICKS;
	}
	
	public void addToPosition(double joyStickVal)
	{
		goToPosition((int)(elevatorMotorA.getSelectedSensorPosition(RobotMap.PRIMARY_PID_LOOP)+Math.floor(joyStickVal*deltaPos)));
		
	}
	
	public void goToPosition(int ticks)
	{
		elevatorMotorA.set(ControlMode.MotionMagic, ticks);
	}
	
	public void setAllMotorsZero()
	{
		elevatorMotorA.set(ControlMode.PercentOutput,0);

		throatMotorA.set(ControlMode.PercentOutput,0);
		throatMotorB.set(ControlMode.PercentOutput,0);
	}
	
	public void setSystemPower(double power)
	{
		elevatorMotorA.set(ControlMode.PercentOutput,power);
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
