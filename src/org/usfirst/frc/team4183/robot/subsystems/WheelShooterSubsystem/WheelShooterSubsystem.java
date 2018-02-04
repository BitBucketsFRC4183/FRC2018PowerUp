package org.usfirst.frc.team4183.robot.subsystems.WheelShooterSubsystem;

import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.BitBucketsSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.SubsystemUtilities.SubsystemTelemetryState;
import org.usfirst.frc.team4183.robot.subsystems.WheelShooterSubsystem.Idle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class WheelShooterSubsystem extends BitBucketsSubsystem {

    private static final int EDGES_PER_ENCODER_COUNT = 1;
	// Put methods for controlling this subsystem
    // here. Call these from Commands.
	private final WPI_TalonSRX leftWheelshooterMotorA;
	private final WPI_TalonSRX rightWheelshooterMotorA;
	
	private final DoubleSolenoid positionChanger;
	
	private final DoubleSolenoid gate;
	
	private FirePos shooterPos = FirePos.LOWSHOT;
	
	private static SendableChooser<SubsystemTelemetryState> telemetryState;
	
	static enum FirePos
	{
		HIGHSHOT(1), LOWSHOT(.4), MANUAL(0);
		
		private final double power;
		
		FirePos(double pow)
		{
			this.power = pow;
		}
		
		public double getPow() {return power;}
	}

	
	public WheelShooterSubsystem() {
		setName("WheelShooterSubsystem");
		
		leftWheelshooterMotorA = new WPI_TalonSRX(RobotMap.WHEEL_SHOOTER_LEFT_1_MOTOR_ID);
		rightWheelshooterMotorA = new WPI_TalonSRX(RobotMap.WHEEL_SHOOTER_RIGHT_1_MOTOR_ID);
		positionChanger = new DoubleSolenoid(RobotMap.WHEEL_SHOOTER_HIGH_POS_CHANNEL, RobotMap.WHEEL_SHOOTER_LOW_POS_CHANNEL);
		gate = new DoubleSolenoid(RobotMap.GATE_OPEN_POS_CHANNEL,RobotMap.GATE_CLOSE_POS_CHANNEL);
		leftWheelshooterMotorA.setInverted(true);
		
		
		telemetryState = new SendableChooser<SubsystemTelemetryState>();
    	
    	telemetryState.addDefault("Off", SubsystemTelemetryState.OFF);
    	telemetryState.addObject( "On",  SubsystemTelemetryState.ON);
    	
    	SmartDashboard.putData("DriveTelemetry", telemetryState);
	}
	//VVV SET THIS TO PRIVATE AND MAKE A PROPER DISABLE COMMAND THIS IS TEMPOARY 
	public void disable() {
		setAllMotorsZero();
		setGateOpen();
		setPosToLow();
		
	}
	
	public void setGateOpen()
	{
		gate.set(DoubleSolenoid.Value.kForward);
	}
	
	public void setGateClose()
	{
		gate.set(DoubleSolenoid.Value.kReverse);
	}
	
	public FirePos getFireSpeedPos()
	{
		return shooterPos;
	}
	
	public void setPosToHigh()
	{
		positionChanger.set(DoubleSolenoid.Value.kForward);
	}
	
	public void setFireSpeedState(FirePos state)
	{
		 shooterPos = state;
	}
	
	public void setPosToLow()
	{
		positionChanger.set(DoubleSolenoid.Value.kReverse);
	}
	
	private void setAllMotorsZero() 
	{
		leftWheelshooterMotorA.set(ControlMode.PercentOutput,0.0);
		//leftWheelshooterMotorB .set(0.0);
		rightWheelshooterMotorA.set(ControlMode.PercentOutput,0.0);
		//rightWheelshooterMotorB.set(0.0);			
		}
	public void setMotorSpeed(double speed) {
		leftWheelshooterMotorA.set(ControlMode.PercentOutput,speed);
		rightWheelshooterMotorA.set(ControlMode.PercentOutput,speed);
	}
	
	
	//implement this properly in order to control the speed of the wheels.
	private void setupClosedLoopMaster( WPI_TalonSRX m) 
	{
		// TODO: New functions provide ErrorCode feedback if there is a problem setting up the controller
		
		m.set(ControlMode.Position,0.0);
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
	
	public double getCurrentMax()
	{
		double leftAMotCurrent = leftWheelshooterMotorA.getOutputCurrent();
		double rightAMotCurrent = rightWheelshooterMotorA.getOutputCurrent();
		
		return Math.max(leftAMotCurrent, rightAMotCurrent);
	}
	
	private int getMotorNativeUnits(WPI_TalonSRX m) {
		return m.getSelectedSensorPosition(RobotMap.PRIMARY_PID_LOOP);
	}
	
	public int getLeftNativeUnits() {
		return getMotorNativeUnits(leftWheelshooterMotorA);
	}
	
	public int getRightNativeUnits() {
		return getMotorNativeUnits(rightWheelshooterMotorA);
	}
	
	private double getMotorEncoderUnits(WPI_TalonSRX m) {
		return getMotorNativeUnits(m)/EDGES_PER_ENCODER_COUNT;
	}
	
	public double getLeftEncoderUnits() {
		return getMotorNativeUnits(leftWheelshooterMotorA);
	}
	
	public double getRightEncoderUnits() {
		return getMotorEncoderUnits(rightWheelshooterMotorA);
	}
	
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new Idle());
    }
	public void diagnosticsFlagSet() {
		// TODO Auto-generated method stub
		
	}
	

	@Override
	public void diagnosticsInit() {
		// TODO Auto-generated method stub
		
	}
	@Override
	public void diagnosticsExecute() {
		// TODO Auto-generated method stub
		
	}
	@Override
	public void diagnosticsCheck() {
		// TODO Auto-generated method stub
		
	}
	@Override
	public void setDiagnosticsFlag(boolean enable) {
		// TODO Auto-generated method stub
		runDiagnostics = enable;
	}
	@Override
	public boolean getDiagnosticsFlag() {
		// TODO Auto-generated method stub
		return runDiagnostics;
	}
	@Override
	public void periodic() {
		SmartDashboard.putString("Shooter Mode", shooterPos.toString());
		// TODO Auto-generated method stub
		if(telemetryState.getSelected() == SubsystemTelemetryState.ON) {

		}
		
		SmartDashboard.putNumber("leftShooter Pow", leftWheelshooterMotorA.get());
		SmartDashboard.putNumber("rightShooter Pow", rightWheelshooterMotorA.get());
		
		
	}
}

