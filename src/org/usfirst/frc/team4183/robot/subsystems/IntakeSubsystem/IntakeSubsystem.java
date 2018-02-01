package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import java.util.ArrayList;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.BitBucketsSubsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class IntakeSubsystem extends BitBucketsSubsystem {
	
	private final WPI_TalonSRX leftIntakeMotor; 
	private final WPI_TalonSRX rightIntakeMotor; 
	private final DoubleSolenoid intakegate;
	
	
	private double timeCurrLimitInit = 0;
	boolean currentLimitAct = false;

	private static ArrayList<WPI_TalonSRX> motors;
	private static ArrayList<DoubleSolenoid> solenoids;

	public IntakeSubsystem() {
		motors = new ArrayList<WPI_TalonSRX>();
		solenoids = new ArrayList<DoubleSolenoid>();
		
		leftIntakeMotor = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_LEFT_ID);
		rightIntakeMotor = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_RIGHT_ID);
		leftIntakeMotor.setInverted(true);
		rightIntakeMotor.setInverted(false);
		leftIntakeMotor.configOpenloopRamp(0.5, RobotMap.CONTROLLER_TIMEOUT_MS);
		rightIntakeMotor.configOpenloopRamp(0.5, RobotMap.CONTROLLER_TIMEOUT_MS);

		intakegate = new DoubleSolenoid(RobotMap.INTAKE_PNEUMA_OPEN_CHANNEL, RobotMap.INTAKE_PNEUMA_CLOSED_CHANNEL);
		
		
		solenoids.add(intakegate);
		
	}
	public boolean getCurrLimitStatus()
	{
		return currentLimitAct;
	}
	public double getTimeCurrentLimit()
	{
		return timeCurrLimitInit;
	}
	public void disable() {
		setAllMotorsZero();
		//closegate();
	}
	
	public void rotatePow(double pow)
	{
		leftIntakeMotor.set(pow);
		rightIntakeMotor.set(-pow);
	}
	
	public void closegate() {
		intakegate.set(DoubleSolenoid.Value.kReverse);
	}
	public void opengate() {
		intakegate.set(DoubleSolenoid.Value.kForward);
	}
	
	private void setAllMotorsZero() 
	{
		leftIntakeMotor.set(ControlMode.PercentOutput, 0.0);
		rightIntakeMotor.set(ControlMode.PercentOutput, 0.0);
	}
	public void setMotorSpeed(double speed) {
		leftIntakeMotor.set(speed);
		rightIntakeMotor.set(speed);
	}
	
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new Idle());
    }
	
	public void checkCurrentLimit(double currTimeInit)
    {
    	if(getCurrentMax() > RobotMap.INTAKE_MAX_CURRENT) {
    		if (!currentLimitAct)
       	 {
       	 timeCurrLimitInit = currTimeInit;}
    		
    		currentLimitAct =true;
    	 
    	}
    	else
    	{
    		currentLimitAct = false;
    		timeCurrLimitInit = 0;
    	}
    }
	
	
	@Override
	public void diagnosticsInit() {
		
		// Checks motors for any current
		for(WPI_TalonSRX talon: motors) {
			if(talon.getOutputCurrent() == 0) {
				
			}
		}
	}
	
	@Override
	public void diagnosticsCheck() {
		
	}
	@Override
	public void periodic() {
		// TODO Auto-generated method stub
		SmartDashboard.putNumber("Intake Current", Robot.intakeSubsystem.getCurrentMax());
	}
	@Override
	public void diagnosticsExecute() {
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
	public double getCurrentMax()
	{
		double leftintakecurrent = leftIntakeMotor.getOutputCurrent(); 
		double rightintakecurrent = rightIntakeMotor.getOutputCurrent();
		
		double average = (leftintakecurrent+rightintakecurrent)/2;
		
		return average;
		}
	}



