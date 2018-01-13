package org.usfirst.frc.team4183.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.commands.IntakeSubsystem.Idle;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class IntakeSubsystem extends Subsystem {
	
	private final WPI_TalonSRX leftintakemotor; 
	private final WPI_TalonSRX rightintakemotor; 
	private final DoubleSolenoid intakegate;

	public IntakeSubsystem() {
		
		leftintakemotor = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_LIEFT_ID);
		rightintakemotor = new WPI_TalonSRX(RobotMap.INTAKE_MOTOR_RIGHT_ID);
		intakegate = new DoubleSolenoid(RobotMap.INTAKE_PNEUMA_OPEN_CHANNEL, RobotMap.INTAKE_PNEUMA_CLOSED_CHANNEL);
		
	}
	public void disable() {
		setAllMotorsZero();
		closegate();
	}
	
	public void closegate() {
		intakegate.set(DoubleSolenoid.Value.kReverse);
	}
	public void opengate() {
		intakegate.set(DoubleSolenoid.Value.kForward);
	}
	
	private void setAllMotorsZero() 
	{
		leftintakemotor.set(0.0);
		rightintakemotor.set(0.0);
	}
	
	public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new Idle());
    }
}

