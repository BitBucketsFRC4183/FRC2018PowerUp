package org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.BitBucketsSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.SubsystemUtilities.DiagnosticsInformation;
import org.usfirst.frc.team4183.robot.subsystems.SubsystemUtilities.DiagnosticsState;
import org.usfirst.frc.team4183.robot.subsystems.SubsystemUtilities.SubsystemTelemetryState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ElevatorSubsystem extends BitBucketsSubsystem {

	private final TalonSRX elevatorMotorA;
	
	private final TalonSRX throatMotorA;
	private final TalonSRX throatMotorB;
	
	private final DoubleSolenoid brakePneu;
	
	private final int UNITS_PER_FEET = 1000;		/// TODO: Remove this and use conversions from RobotMap
		
	private static SendableChooser<SubsystemTelemetryState> telemetryState;
	
	Faults elevatorMotorAFaults = new Faults();
	
	private int test_elevator_ticks = 0;
	
	private int currentTicks;
		
	public static enum ElevatorPresets
	{
		BOTTOM(1), MIDDLE(30000), HIGH(66000), TOP(93500);
		//BOTTOM(100), MIDDLE(47500), HIGH(117000), TOP(150000);
		
		private int nativeTicks;
		ElevatorPresets(int nativeTicks)
		{
			this.nativeTicks = nativeTicks;
		}
		
		public double getPosition_in() {
			return this.nativeTicks / RobotMap.ELEVATOR_INCHES_PER_NATIVE_TICKS;
		}
		
		public int getNativeTicks()
		{
			return this.nativeTicks;
		}
	}
	
	public void setCurrentTicks(int desiredTicks)
	{
		currentTicks = desiredTicks;
	}
	
	public int getCurrentSetTicks()
	{
		return currentTicks;
	}
	
	private static double testModePeriod_sec = 2.0;
	
	public static int holdUnits = 0;
	
	//adjust this later for the driver control
	private final int deltaPos = UNITS_PER_FEET;
		
	//This is used to open the intake mandible if the position is too close.

	public ElevatorSubsystem()
	{
		this.setName("ElevatorSubsystem");
		
		currentTicks = ElevatorPresets.BOTTOM.getNativeTicks() - 1;
		
		elevatorMotorA = new TalonSRX(RobotMap.ELEVATOR_MOTOR_A_ID);
		elevatorMotorA.setSensorPhase(RobotMap.ELEVATOR_MOTOR_SENSOR_PHASE);
		elevatorMotorA.setInverted(RobotMap.ELEVATOR_MOTOR_INVERSION);
		
		throatMotorA = new TalonSRX(RobotMap.THROAT_MOTOR_LEFT_ID);
		throatMotorA.setInverted(true);
		throatMotorB = new TalonSRX(RobotMap.THROAT_MOTOR_RIGHT_ID);
		
		brakePneu = new DoubleSolenoid(RobotMap.ELEVATOR_PNEUMA_BRAKE_CLOSE_CHANNEL,RobotMap.ELEVATOR_PNEUMA_BRAKE_OPEN_CHANNEL);
		
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
		
		elevatorMotorA.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,0);
		
		elevatorMotorA.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen,0);
		
		elevatorMotorA.overrideLimitSwitchesEnable(true);
		
		
		elevatorMotorA.setNeutralMode(NeutralMode.Brake);
		
		elevatorMotorA.setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_LOOP, RobotMap.CONTROLLER_TIMEOUT_MS);
		
		DIAG_LOOPS_RUN = 10;
		
		telemetryState = new SendableChooser<SubsystemTelemetryState>();
    	telemetryState.addDefault("Off", SubsystemTelemetryState.OFF);
    	telemetryState.addObject( "On",  SubsystemTelemetryState.ON);
    	
    	SmartDashboard.putData("ElevatorTelemetry", telemetryState);
		
		setAllMotorsZero();
	}
	
	public void disableThroat()
	{
		throatMotorA.set(ControlMode.PercentOutput,0);
		throatMotorB.set(ControlMode.PercentOutput,0);
	}
		
	public boolean outputDangerZoneInfo()
	{
		double currPos = Robot.elevatorSubsystem.getElevatorNativeUnits();
		double cmd = Robot.oi.elevatorJoystick.get();
		boolean dangerZone = (currPos < RobotMap.ELEVATOR_DANGER_ZONE);
		
		boolean restrictCmd = (dangerZone && (cmd < 0));
		SmartDashboard.putBoolean("Dangerzone", dangerZone);
		SmartDashboard.putBoolean("Restrict Command", restrictCmd);
		return dangerZone;
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
			holdPosition(holdUnits);
		}
		else
		{
			holdUnits = 0;
		}
	}
	
	//return true if a cube is present
	
	public void releasePos()
	{
		holdEncodPos(false);
	}
	
	//this one engages the brake
	public void holdPos()
	{
		holdEncodPos(true);
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
	
	public void addToPosition(double joyStickVal) /// TODO: Change concept to small rate control then capture position and hold
	{											  /// otherwise the operator will have to continue to hold joystick and elevator will move
		holdPosition((int)(elevatorMotorA.getSelectedSensorPosition(RobotMap.PRIMARY_PID_LOOP)+Math.floor(joyStickVal*deltaPos)));
		
	}
		
	public void setAllMotorsZero()
	{
		elevatorMotorA.set(ControlMode.PercentOutput,0);

		throatMotorA.set(ControlMode.PercentOutput,0);
		throatMotorB.set(ControlMode.PercentOutput,0);
	}

	public void holdPosition(int ticks)
	{
		elevatorMotorA.set(ControlMode.MotionMagic,ticks);
	}
	
	public void setSystemPower(double power)
	{
		elevatorMotorA.set(ControlMode.PercentOutput,power);
	}
	
	private double getMotorNativeUnits(TalonSRX m) {
		return elevatorMotorA.getSelectedSensorPosition(RobotMap.PRIMARY_PID_LOOP);
	}
	
	//Output in native units per 100ms
	public double getElevatorVelocity()
	{
		return elevatorMotorA.getSelectedSensorVelocity(RobotMap.PRIMARY_PID_LOOP);
	}

	public double getElevatorNativeUnits() {
		return getMotorNativeUnits(elevatorMotorA);
	}
	
	public double getElevatorCurrent() {
		return elevatorMotorA.getOutputCurrent();
	}
	
	public boolean getElevatorLimitSwitch() {
		return elevatorMotorA.getSensorCollection().isRevLimitSwitchClosed();
	}
	
	// Limits the value to the provided limit, maintaining input sign
	// sign(value) * min(|value|, limit)
	public double limitJoystickCommand(double value, double limit) {
		return Math.signum(value) * Math.min(Math.abs(value), limit);
	}
	
	// Move is complete when we are within tolerance and can consider starting the next move
	public boolean isMoveComplete(int targetTicks)	// At timeout should be used with this
	{
		int error = (int) Math.abs(targetTicks - elevatorMotorA.getSelectedSensorPosition(RobotMap.PRIMARY_PID_LOOP));
		return (error  < RobotMap.ELEVATOR_POSITION_TOLERANCE_NATIVE_TICKS);
	}
	
	
	public double getTestModePeriod_sec()
	{
		return testModePeriod_sec;
	}

	@Override
	public void diagnosticsInit() {
		test_elevator_ticks = (int)getElevatorNativeUnits();
		holdPosition((int)RobotMap.ELEVATOR_TEST_NATIVE_UNITS+test_elevator_ticks);
	}
	
	@Override
	public void diagnosticsExecute() {
	}
	
	@Override
	public void diagnosticsCheck() {
		/* Reset Flag */
		runDiagnostics = false;
		
		/* Diagnostics */
		lastKnownState = DiagnosticsState.PASS;
		SmartDashboard.putBoolean(getName() + "Diagnostics", true); // Innocent until proven guilt
		
		
		if(Robot.diagInformation.getSelected() == DiagnosticsInformation.SUBSYSTEM_EXTENDED) {
			SmartDashboard.putBoolean("ElevatorMotor", true);
		}
		if(getElevatorNativeUnits() < test_elevator_ticks+RobotMap.ELEVATOR_TEST_NATIVE_UNITS/2) {
			SmartDashboard.putBoolean(getName() + "Diagnostics", false);
			lastKnownState = DiagnosticsState.FAIL;
			if(Robot.diagInformation.getSelected() == DiagnosticsInformation.SUBSYSTEM_EXTENDED)
				SmartDashboard.putBoolean("ElevatorMotor", false);
			setSystemPower(0);
		}
		else
		{
			holdPosition(test_elevator_ticks);
		}
		
		
		
	}

	@Override
	protected void initDefaultCommand() {
		// Not here, use initialize and explicit start instead of setDefaultCommand(new Idle());
		// this prevent problems when using the subsystem from an autonomous mode
	}



	@Override
	public void periodic() {
		outputDangerZoneInfo();
		if(telemetryState.getSelected() == SubsystemTelemetryState.ON) {
			SmartDashboard.putNumber("ElevatorPosition", getElevatorNativeUnits());
			SmartDashboard.putNumber("ElevatorCurrent", elevatorMotorA.getOutputCurrent());
			SmartDashboard.putBoolean("Forward Limit Switch",  elevatorMotorA.getSensorCollection().isFwdLimitSwitchClosed());
			//SmartDashboard.putBoolean("Reverse Limit Switch", elevatorMotorAFaults.ReverseLimitSwitch);
			SmartDashboard.putBoolean("Reverse Limit Switch", elevatorMotorA.getSensorCollection().isRevLimitSwitchClosed());
			SmartDashboard.putNumber("Elevator Velocity", getElevatorVelocity());

		}
		elevatorMotorA.getFaults(elevatorMotorAFaults);

		// Reset the encoder to zero each time the limit switch is reached
		if(elevatorMotorA.getSensorCollection().isRevLimitSwitchClosed())
		{
			elevatorMotorA.setSelectedSensorPosition(0, RobotMap.PRIMARY_PID_LOOP, RobotMap.CONTROLLER_TIMEOUT_MS);
			
			// In all modes except auto we want to reset the hold position to here
			if (Robot.runMode != Robot.RunMode.AUTO)
			{
				new Idle();
			}
		}
		
		
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

	public void initialize() {
		// TODO Auto-generated method stub
		Idle initialCommand = new Idle();
		initialCommand.start();
		
	}
	
	
}
