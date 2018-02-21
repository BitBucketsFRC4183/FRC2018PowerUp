package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import java.util.ArrayList;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.BitBucketsSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class IntakeSubsystem extends BitBucketsSubsystem {
	
	private final WPI_TalonSRX leftIntakeMotor; 
	private final WPI_TalonSRX rightIntakeMotor; 
	private final TalonSRX throatMotorA;
	private final TalonSRX throatMotorB;
	
	private final DoubleSolenoid intakegate;
	
	private final DigitalInput leftMaxLimit;
	private final DigitalInput leftMinLimit;
	private final DigitalInput rightMaxLimit;
	private final DigitalInput rightMinLimit;
	
	private double timeCurrLimitInit = 0;
	boolean currentLimitAct = false;
	
	boolean lastCurrentState = false;

	private static ArrayList<WPI_TalonSRX> motors;
	private static ArrayList<DoubleSolenoid> solenoids;

	public IntakeSubsystem() {
		motors = new ArrayList<WPI_TalonSRX>();
		solenoids = new ArrayList<DoubleSolenoid>();
		
		leftIntakeMotor = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_LEFT_ID);
		rightIntakeMotor = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_RIGHT_ID);
		
		throatMotorA = new TalonSRX(RobotMap.THROAT_MOTOR_LEFT_ID);
		throatMotorA.setInverted(true);
		throatMotorB = new TalonSRX(RobotMap.THROAT_MOTOR_RIGHT_ID);
		
		leftIntakeMotor.setInverted(true);
		rightIntakeMotor.setInverted(false);
		leftIntakeMotor.configOpenloopRamp(0.5, RobotMap.CONTROLLER_TIMEOUT_MS);
		rightIntakeMotor.configOpenloopRamp(0.5, RobotMap.CONTROLLER_TIMEOUT_MS);

		intakegate = new DoubleSolenoid(RobotMap.INTAKE_PNEUMA_OPEN_CHANNEL, RobotMap.INTAKE_PNEUMA_CLOSED_CHANNEL);
		
		leftMaxLimit = new DigitalInput(RobotMap.INTAKE_LIMIT_LEFT_MAX_ID);
		leftMinLimit = new DigitalInput(RobotMap.INTAKE_LIMIT_LEFT_MIN_ID);
		rightMaxLimit = new DigitalInput(RobotMap.INTAKE_LIMIT_RIGHT_MAX_ID);
		rightMinLimit = new DigitalInput(RobotMap.INTAKE_LIMIT_RIGHT_MIN_ID);
		
		
		solenoids.add(intakegate);
		
	}
	
	//checks to see if the either of the Min Limit Switches are hit for the pneumatics
	public boolean getMinLimit()
	{
		if (leftMinLimit.get() || rightMinLimit.get())
		{
			return true;
		}
		return false;
	}
	
	//checks to see if either of the Max Limit Switches are hit for the pneumatics
	public boolean getMaxLimit()
	{
		if (leftMaxLimit.get() || rightMaxLimit.get())
		{
			return true;
		}
		return false;
	}
	
	public boolean getLastCurrent()
	{
		return lastCurrentState;
	}
	public boolean getCurrLimitStatus()
	{
		return currentLimitAct;
	}
	public void disable() {
		setAllMotorsZero();
    	closeMandible();
	}
	
	public void rotatePow(double pow)
	{
		leftIntakeMotor.set(pow);
		rightIntakeMotor.set(-pow);
	}
	
	public void closeMandible() {
		intakegate.set(DoubleSolenoid.Value.kReverse);
	}
	public void openMandible() {
		intakegate.set(DoubleSolenoid.Value.kForward);
	}
	
	private void setAllMotorsZero() 
	{
		leftIntakeMotor.set(ControlMode.PercentOutput, 0.0);
		rightIntakeMotor.set(ControlMode.PercentOutput, 0.0);
		throatMotorA.set(ControlMode.PercentOutput, 0.0);
		throatMotorB.set(ControlMode.PercentOutput, 0.0);
	}
	public void setLeftMotorSpeed(double speed) {
		leftIntakeMotor.set(ControlMode.PercentOutput, speed);
		throatMotorA.set(ControlMode.PercentOutput, speed);
	}
	
	public void setRightMotorSpeed(double speed) {
		rightIntakeMotor.set(ControlMode.PercentOutput, speed);
		throatMotorB.set(ControlMode.PercentOutput, speed);
	}
	
	public void setIntakeMotorSpeed(double speed) {
		setLeftMotorSpeed(speed);
		setRightMotorSpeed(speed);
	}
	
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new Idle());
    }
	
	public boolean checkCurrentLimit(double currTimeInit)
    {
    	if(getCurrentMax() > RobotMap.INTAKE_MAX_CURRENT) {
    		if (!currentLimitAct)
       	 {
       	 timeCurrLimitInit = currTimeInit;
       	 }
    		currentLimitAct =true;
    		if (currTimeInit - timeCurrLimitInit > .25)
    		{
    			lastCurrentState = true;
    			return true;
    		}
    		return false;
    	}
    	else
    	{
    		lastCurrentState = false;
    		currentLimitAct = false;
    		timeCurrLimitInit = 0;
    		return false;
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
		SmartDashboard.putBoolean("Min Limit Intake", Robot.intakeSubsystem.getMinLimit());
		SmartDashboard.putBoolean("Max Limit Intake", Robot.intakeSubsystem.getMaxLimit());
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



