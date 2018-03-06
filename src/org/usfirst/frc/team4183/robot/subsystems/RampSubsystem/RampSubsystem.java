package org.usfirst.frc.team4183.robot.subsystems.RampSubsystem;

import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.BitBucketsSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Servo;

public class RampSubsystem extends BitBucketsSubsystem{
	private WPI_TalonSRX rightRampMotor;
	private WPI_TalonSRX leftRampMotor;
	private Servo rampReleaseServ;

	public RampSubsystem()
	{
		this.setName("RampSubsystem");
		leftRampMotor = new WPI_TalonSRX(RobotMap.LEFT_RAMP_MOTOR_ID);
		rightRampMotor = new WPI_TalonSRX(RobotMap.RIGHT_RAMP_MOTOR_ID);
		rampReleaseServ = new Servo(RobotMap.RAMP_RELEASE_SERVO_ID);
	}
	
	public void setRampSpeed(double leftSpeed, double rightSpeed)
	{
		leftRampMotor.set(ControlMode.PercentOutput, leftSpeed);
		rightRampMotor.set(ControlMode.PercentOutput, rightSpeed);
	}
	
	public void disabled()
	{
		leftRampMotor.set(ControlMode.PercentOutput,0);
		rightRampMotor.set(ControlMode.PercentOutput, 0);
	}
	
	public void closeRamp()
	{
		rampReleaseServ.setAngle(180);
	}
	
	public void releaseRamp()
	{
		rampReleaseServ.setAngle(90);
	}
	@Override
	public void diagnosticsInit() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void diagnosticsCheck() {
		// TODO Auto-generated method stub
		
	}
	
	@Override
	protected void initDefaultCommand() {
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
		// TODO Auto-generated method stub
		
	}

	@Override
	public boolean getDiagnosticsFlag() {
		// TODO Auto-generated method stub
		return false;
	}

}
