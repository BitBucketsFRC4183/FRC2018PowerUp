package org.usfirst.frc.team4183.robot.commands.subsystems;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends Subsystem{
	public CANTalon leftFrontMot;
	public CANTalon leftBackMot;
	public CANTalon rightFrontMot;
	public CANTalon rightBackMot;
	
	SpeedControllerGroup m_left = new SpeedControllerGroup(leftFrontMot, leftBackMot);
	
	public DifferentialDrive m_drive = new DifferentialDrive();
	
	
	
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

}
