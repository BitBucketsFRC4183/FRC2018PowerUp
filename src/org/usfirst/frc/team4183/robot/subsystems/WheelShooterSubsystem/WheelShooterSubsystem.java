package org.usfirst.frc.team4183.robot.subsystems.WheelShooterSubsystem;

import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.BitBucketsSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.WheelShooterSubsystem.Idle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 *
 */
public class WheelShooterSubsystem extends BitBucketsSubsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private final WPI_TalonSRX leftWheelshooterMotorA;
	private final WPI_TalonSRX leftWheelshooterMotorB;
	private final WPI_TalonSRX rightWheelshooterMotorA;
	private final WPI_TalonSRX rightWheelshooterMotorB;
	
	private final DoubleSolenoid positionChanger;
	
	private final DoubleSolenoid gate;
	
	private FirePos shooterPos = FirePos.LOWSHOT;
	
	static enum FirePos
	{
		HIGHSHOT(.8), LOWSHOT(.4), MANUAL(0);
		
		private final double power;
		
		FirePos(double pow)
		{
			this.power = pow;
		}
		
		public double getPow() {return power;}
	}

	
	private boolean present = false;
	
	public WheelShooterSubsystem() {
		leftWheelshooterMotorA = new WPI_TalonSRX(RobotMap.WHEEL_SHOOTER_LEFT_1_MOTOR_ID);
		leftWheelshooterMotorB = new WPI_TalonSRX(RobotMap.WHEEL_SHOOTER_LEFT_2_MOTOR_ID);
		rightWheelshooterMotorA = new WPI_TalonSRX(RobotMap.WHEEL_SHOOTER_RIGHT_1_MOTOR_ID);
		rightWheelshooterMotorB = new WPI_TalonSRX(RobotMap.WHEEL_SHOOTER_RIGHT_2_MOTOR_ID);
		positionChanger = new DoubleSolenoid(RobotMap.WHEEL_SHOOTER_HIGH_POS_CHANNEL, RobotMap.WHEEL_SHOOTER_LOW_POS_CHANNEL);
		gate = new DoubleSolenoid(RobotMap.GATE_OPEN_POS_CHANNEL,RobotMap.GATE_CLOSE_POS_CHANNEL);
		leftWheelshooterMotorA.setInverted(true);
		leftWheelshooterMotorB.setInverted(true) ;
		leftWheelshooterMotorB.set(ControlMode.Follower, RobotMap.WHEEL_SHOOTER_LEFT_1_MOTOR_ID);
		rightWheelshooterMotorB.set(ControlMode.Follower, RobotMap.WHEEL_SHOOTER_RIGHT_1_MOTOR_ID);
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
		leftWheelshooterMotorB.set(ControlMode.Follower, RobotMap.WHEEL_SHOOTER_LEFT_1_MOTOR_ID);
		rightWheelshooterMotorB.set(ControlMode.Follower, RobotMap.WHEEL_SHOOTER_RIGHT_1_MOTOR_ID);
		//rightWheelshooterMotorB.set(0.0);			
		}
	public void setMotorSpeed(double speed) {
		leftWheelshooterMotorA.set(ControlMode.PercentOutput,speed);
		rightWheelshooterMotorA.set(ControlMode.PercentOutput,speed);
		leftWheelshooterMotorB.set(ControlMode.Follower, RobotMap.WHEEL_SHOOTER_LEFT_1_MOTOR_ID);
		rightWheelshooterMotorB.set(ControlMode.Follower, RobotMap.WHEEL_SHOOTER_RIGHT_1_MOTOR_ID);
	}
	
	public double getCurrentMax()
	{
		double leftAMotCurrent = leftWheelshooterMotorA.getOutputCurrent();
		double leftBMotCurrent = leftWheelshooterMotorB.getOutputCurrent();
		double rightAMotCurrent = rightWheelshooterMotorA.getOutputCurrent();
		double rightBMotCurrent = rightWheelshooterMotorB.getOutputCurrent();
		
		double leftMax = Math.max(leftAMotCurrent, leftBMotCurrent);
		double rightMax = Math.max(rightAMotCurrent, rightBMotCurrent);
		
		if (leftMax > rightMax)
		{
			return leftMax;
		}
		else if (rightMax > leftMax)
		{
			return rightMax;
		}
		else
		{
			return rightMax;
		}
	}
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new Idle());
    }
	public void diagnosticsFlagSet() {
		// TODO Auto-generated method stub
		
	}
	public void setPresent(boolean b) {
		present = b;
		
	}
	public boolean isPresent() {
		
		return present;
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
		// TODO Auto-generated method stub
		
	}
}

