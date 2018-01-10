package org.usfirst.frc.team4183.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.robot.commands.DriveSubsystem.Idle;

public class DriveSubsystem extends Subsystem
{
	private final WPI_TalonSRX leftMotorFront;		// User follower mode
	private final WPI_TalonSRX leftMotorBack;

	private final WPI_TalonSRX rightMotorFront;		// Use follower mode
	private final WPI_TalonSRX rightMotorBack;

	private final DifferentialDrive drive;
	
    public DriveSubsystem()
    {
    	leftMotorFront = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_FRONT_ID);
    	leftMotorBack = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_BACK_ID);
    	
    	// Use follower mode to minimize shearing commands that could occur if
    	// separate commands are sent to each motor in a group
    	leftMotorBack.set(ControlMode.Follower, RobotMap.LEFT_DRIVE_MOTOR_FRONT_ID);
    	
    	rightMotorFront  = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_FRONT_ID);
    	rightMotorBack   = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_FRONT_ID);

    	// Use follower mode to minimize shearing commands that could occur if
    	// separate commands are sent to each motor in a group
    	rightMotorBack.set(ControlMode.Follower, RobotMap.RIGHT_DRIVE_MOTOR_FRONT_ID);

    	// The differential drive simply requires a left and right speed controller
    	// In this case we can use a single motor controller type on each side
    	// rather than a SpeedControllerGroup because we set up the follower mode
    	// NOTE: We will want to test that this really works, but it should prevent
    	// the loop in the SpeedControllerGroup from being sheared by preemption on
    	// a side that could cause the motors to have different commands on rapid
    	// reverse boundaries (which is hard on the gears, even for 20 ms).
    	// The left vs right can still be sheared by preemption but that is generally 
    	// less harmful on the entire robot as forces and slippage will be absorbed
    	// through the tires and frame (not JUST the gearbox)
    	drive = new DifferentialDrive(leftMotorFront, rightMotorFront);
    	
    	// Now get the other modes set up
    	setBraking(true);
    }
		
	
	@Override
	protected void initDefaultCommand() 
	{
		setDefaultCommand(new Idle());		
		
	}

	public void disable() {
		setAllMotorsZero();
	}
	
	private void setAllMotorsZero() 
	{
		leftMotorFront.set(0.0);
		leftMotorBack.set(0.0);
		rightMotorFront.set(0.0);
		rightMotorBack.set(0.0);			
	}
	

	private void setBraking( boolean setting) 
	{
		/// TODO: Find the correct command in the new motor controller
//		leftMotorFront.enableBrakeMode(setting);
//		leftMotorBack.enableBrakeMode(setting);
//		rightMotorFront.enableBrakeMode(setting);
//		rightMotorBack.enableBrakeMode(setting);
	}

}
