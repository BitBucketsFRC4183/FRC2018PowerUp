package org.usfirst.frc.team4183.robot.subsystems.ClimberSubsystem;

import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.subsystems.ClimberSubsystem.Idle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ClimberSubsystem extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	private TalonSRX climbMotorA;
	private TalonSRX climbMotorB;

	public ClimberSubsystem() {
		climbMotorA = new TalonSRX(RobotMap.CLIMBER_MOTOR_A);
		climbMotorB = new TalonSRX(RobotMap.CLIMBER_MOTOR_B);

		climbMotorA.setInverted(RobotMap.CLIMBER_MOTOR_A_INVERSION_FLAG);
		climbMotorB.setInverted(RobotMap.CLIMBER_MOTOR_B_INVERSION_FLAG);
		
	}
	
	public void setClimberPower(double power) {
		climbMotorA.set(ControlMode.PercentOutput, power);
		climbMotorB.set(ControlMode.PercentOutput, power);
	}
	
	public void initialize() {
		Idle initialCommand = new Idle();
		initialCommand.start();
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

