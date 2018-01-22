package org.usfirst.frc.team4183.robot.subsystems.WheelShooterSubsystem;

import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.BitBucketsSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.WheelShooterSubsystem.Idle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

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
	private boolean present = false;
	
	public WheelShooterSubsystem() {
		leftWheelshooterMotorA = new WPI_TalonSRX(RobotMap.WHEEL_SHOOTER_LEFT_1_MOTOR_ID);
		leftWheelshooterMotorB = new WPI_TalonSRX(RobotMap.WHEEL_SHOOTER_LEFT_2_MOTOR_ID);
		rightWheelshooterMotorA = new WPI_TalonSRX(RobotMap.WHEEL_SHOOTER_RIGHT_1_MOTOR_ID);
		rightWheelshooterMotorB = new WPI_TalonSRX(RobotMap.WHEEL_SHOOTER_RIGHT_2_MOTOR_ID);
		leftWheelshooterMotorA.setInverted(true);
		leftWheelshooterMotorB.setInverted(true) ;
		leftWheelshooterMotorB.set(ControlMode.Follower, RobotMap.WHEEL_SHOOTER_LEFT_1_MOTOR_ID);
		rightWheelshooterMotorB.set(ControlMode.Follower, RobotMap.WHEEL_SHOOTER_RIGHT_1_MOTOR_ID);
	}
	//VVV SET THIS TO PRIVATE AND MAKE A PROPER DISABLE COMMAND THIS IS TEMPOARY 
	public void disable() {
		setAllMotorsZero();
		
	}
	
	private void setAllMotorsZero() 
	{
		leftWheelshooterMotorA.set(0.0);
		//leftWheelshooterMotorB .set(0.0);
		rightWheelshooterMotorA.set(0.0);
		//rightWheelshooterMotorB.set(0.0);			
	}
	public void setMotorSpeed(double speed) {
		leftWheelshooterMotorA.set(speed);
		rightWheelshooterMotorA.set(speed);
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

