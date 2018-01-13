/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4183.robot;

import org.usfirst.frc.team4183.robot.Robot.RunMode;
import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {

	// Use this runMode variable to determine the 
	// current running mode of the Robot.
	public enum RunMode { DISABLED, AUTO, TELEOP, TEST };
	public static RunMode runMode = RunMode.DISABLED;
	
	
	public static DriveSubsystem driveSubsystem;
	public static IntakeSubsystem intakeSubsystem; 
	public static OI oi;
	
	public static LightingControl lightingControl;	
	public static NavxIMU imu;
	
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = OI.instance();
		
		driveSubsystem = new DriveSubsystem();
		intakeSubsystem = new IntakeSubsystem();
		
		imu = new NavxIMU();
		lightingControl = new LightingControl();
		
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
	}
	
	@Override
	public void disabledInit() {
		runMode = RunMode.DISABLED;

		// Set up OI for disabled mode
		oi.setDisabledMode();
		
		// Clear out the scheduler.
		// Will result in only Default Commands (==Idle-s) running,
		// effectively forcing all State Machines into Idle state.
		Scheduler.getInstance().removeAll();
	}

	@Override
	public void autonomousInit() {
		runMode = RunMode.AUTO;
		
		oi.setAutoMode();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() 
	{
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		runMode = RunMode.TELEOP;

		// Set up OI for teleop mode
		oi.setTeleopMode();
	}
	
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		Scheduler.getInstance().run();
	}
}
