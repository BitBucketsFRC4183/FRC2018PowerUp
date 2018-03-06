package org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import java.util.ArrayList;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.BitBucketsSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.SubsystemUtilities.DiagnosticsInformation;
import org.usfirst.frc.team4183.robot.subsystems.SubsystemUtilities.DiagnosticsState;
import org.usfirst.frc.team4183.robot.subsystems.SubsystemUtilities.SubsystemTelemetryState;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
	
	private static SendableChooser<SubsystemTelemetryState> telemetryState;

	public IntakeSubsystem() {
		this.setName("IntakeSubsystem");
		motors = new ArrayList<WPI_TalonSRX>();
		solenoids = new ArrayList<DoubleSolenoid>();
		
		DIAG_LOOPS_RUN = 10;
		
		leftIntakeMotor = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_LEFT_ID);
		rightIntakeMotor = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_RIGHT_ID);
		
		throatMotorA = new TalonSRX(RobotMap.THROAT_MOTOR_LEFT_ID);
		throatMotorA.setInverted(true);
		throatMotorB = new TalonSRX(RobotMap.THROAT_MOTOR_RIGHT_ID);
		
		setNeutral(NeutralMode.Coast);
		
		leftIntakeMotor.setInverted(true);
		rightIntakeMotor.setInverted(false);
		
		throatMotorA.configNeutralDeadband(0.0, RobotMap.CONTROLLER_TIMEOUT_MS);
		throatMotorB.configNeutralDeadband(0.0, RobotMap.CONTROLLER_TIMEOUT_MS);
		
		leftIntakeMotor.configOpenloopRamp(0.5, RobotMap.CONTROLLER_TIMEOUT_MS);
		rightIntakeMotor.configOpenloopRamp(0.5, RobotMap.CONTROLLER_TIMEOUT_MS);
		throatMotorA.configOpenloopRamp(0.0, RobotMap.CONTROLLER_TIMEOUT_MS);
		throatMotorB.configOpenloopRamp(0.0, RobotMap.CONTROLLER_TIMEOUT_MS);

		intakegate = new DoubleSolenoid(RobotMap.INTAKE_PNEUMA_OPEN_CHANNEL, RobotMap.INTAKE_PNEUMA_CLOSED_CHANNEL);
		
		leftMaxLimit = new DigitalInput(RobotMap.INTAKE_LIMIT_LEFT_MAX_ID);
		leftMinLimit = new DigitalInput(RobotMap.INTAKE_LIMIT_LEFT_MIN_ID);
		rightMaxLimit = new DigitalInput(RobotMap.INTAKE_LIMIT_RIGHT_MAX_ID);
		rightMinLimit = new DigitalInput(RobotMap.INTAKE_LIMIT_RIGHT_MIN_ID);
		
		
		solenoids.add(intakegate);
		
		telemetryState = new SendableChooser<SubsystemTelemetryState>();
    	telemetryState.addDefault("Off", SubsystemTelemetryState.OFF);
    	telemetryState.addObject( "On",  SubsystemTelemetryState.ON);
    	
    	SmartDashboard.putData("IntakeTelemetry", telemetryState);
		
	}
	
	private void setNeutral(NeutralMode neutralMode) 
	{	
		leftIntakeMotor.setNeutralMode(neutralMode);
		rightIntakeMotor.setNeutralMode(neutralMode);
		throatMotorA.setNeutralMode(neutralMode);
		throatMotorB.setNeutralMode(neutralMode);
		
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
	public void setLeftThroatSpeed(double speed) {
		throatMotorA.set(ControlMode.PercentOutput, speed);
	}
	
	public void setRightThroatSpeed(double speed) {
		throatMotorB.set(ControlMode.PercentOutput, speed);
	}
	
	public void setLeftMotorSpeed(double intakeSpeed, double throatSpeed) {
		leftIntakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
		setLeftThroatSpeed(throatSpeed);
	}
	
	public void setRightMotorSpeed(double intakeSpeed, double throatSpeed) {
		rightIntakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
		setRightThroatSpeed(throatSpeed);
	}
	
	public void setIntakeMotorsToSpeed(double intakeSpeed, double throatSpeed) {
		setLeftMotorSpeed(intakeSpeed, throatSpeed);
		setRightMotorSpeed(intakeSpeed, throatSpeed);
	}
	
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new Idle());
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
		//TODO: Might want to move this but I figure opening pneumatics first is safest
		openMandible();
	}
	
	@Override
	public void diagnosticsExecute() {
		setIntakeMotorsToSpeed(RobotMap.MOTOR_TEST_PERCENT, RobotMap.MOTOR_TEST_PERCENT);
	}
	
	@Override
	public void diagnosticsCheck() {
		/* Reset Flag */
		runDiagnostics = false;
		
		/* Diagnostics */
		lastKnownState = DiagnosticsState.PASS; // Passes unless we find a fault
		SmartDashboard.putBoolean(getName() + "Diagnostics", true);
		
		if(Robot.diagInformation.getSelected() == DiagnosticsInformation.SUBSYSTEM_EXTENDED)
		{
			SmartDashboard.putBoolean("ThroatMotorA", true);
			SmartDashboard.putBoolean("ThroatMotorB", true);
			SmartDashboard.putBoolean("IntakeLeft",  true);
			SmartDashboard.putBoolean("IntakeRight", true);
		}
		
		if(throatMotorA.getOutputCurrent() <= RobotMap.MINIMUM_MOTOR_CURR) {
			if(Robot.diagInformation.getSelected() == DiagnosticsInformation.SUBSYSTEM_EXTENDED)
				SmartDashboard.putBoolean("ThroatMotorA", false);
			SmartDashboard.putBoolean(getName() + "Diagnostics", false);
			lastKnownState = DiagnosticsState.FAIL;
		}
		throatMotorA.set(ControlMode.PercentOutput, 0.0);
		
		if(throatMotorB.getOutputCurrent() <= RobotMap.MINIMUM_MOTOR_CURR) {
			if(Robot.diagInformation.getSelected() == DiagnosticsInformation.SUBSYSTEM_EXTENDED)
				SmartDashboard.putBoolean("ThroatMotorB", false);
			SmartDashboard.putBoolean(getName() + "Diagnostics", false);
			lastKnownState = DiagnosticsState.FAIL;
		}
		throatMotorB.set(ControlMode.PercentOutput, 0.0);
		
		if(leftIntakeMotor.getOutputCurrent() <= RobotMap.MINIMUM_MOTOR_CURR) {
			if(Robot.diagInformation.getSelected() == DiagnosticsInformation.SUBSYSTEM_EXTENDED)
				SmartDashboard.putBoolean("IntakeLeft", false);
			SmartDashboard.putBoolean(getName() + "Diagnostics", false);
			lastKnownState = DiagnosticsState.FAIL;
		}
		leftIntakeMotor.set(ControlMode.PercentOutput, 0.0);
		
		if(rightIntakeMotor.getOutputCurrent() <= RobotMap.MINIMUM_MOTOR_CURR) {
			if(Robot.diagInformation.getSelected() == DiagnosticsInformation.SUBSYSTEM_EXTENDED)
				SmartDashboard.putBoolean("IntakeRight", false);
			SmartDashboard.putBoolean(getName() + "Diagnostics", false);
			lastKnownState = DiagnosticsState.FAIL;
		}
		rightIntakeMotor.set(ControlMode.PercentOutput, 0.0);
	}
	@Override
	public void periodic() {
		// TODO Auto-generated method stub
		
		if(telemetryState.getSelected() == SubsystemTelemetryState.ON) {
			SmartDashboard.putNumber("Intake Current", Robot.intakeSubsystem.getCurrentMax());
			SmartDashboard.putBoolean("Min Limit Intake", Robot.intakeSubsystem.getMinLimit());
			SmartDashboard.putBoolean("Max Limit Intake", Robot.intakeSubsystem.getMaxLimit());
			
			SmartDashboard.putNumber("ThroatMotorA", throatMotorA.getOutputCurrent());
			SmartDashboard.putNumber("ThroatMotorB", throatMotorB.getOutputCurrent());
			SmartDashboard.putNumber("LeftIntakeMotor", leftIntakeMotor.getOutputCurrent());
			SmartDashboard.putNumber("RightIntakeMotor", rightIntakeMotor.getOutputCurrent());
		}
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

	public void initialize() {
		// TODO Auto-generated method stub
		Idle initialCommand = new Idle();
		initialCommand.start();
		
	}
}



