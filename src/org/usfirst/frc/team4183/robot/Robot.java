/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc.team4183.robot;
import java.util.HashSet;
import java.util.Set;
import org.usfirst.frc.team4183.robot.Robot.RunMode;
import org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem.RunScript;
import org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem.AutonomousSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem.DriveSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.RampSubsystem.RampSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.VisionSubsystem.VisionSubsystem;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

import org.usfirst.frc.team4183.utils.DoEveryN;
import org.usfirst.frc.team4183.utils.Stopwatch;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;

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
	public static RunMode lastState = runMode;	
	

	// All robots will have these subsystems
	// Other may be optional or interchangable
	public static DriveSubsystem driveSubsystem;
	public static IntakeSubsystem intakeSubsystem;
	public static AutonomousSubsystem autonomousSubsystem;
	public static VisionSubsystem visionSubsystem;
	public static RampSubsystem rampSubsystem;
	public static ElevatorSubsystem elevatorSubsystem;
	
	
	// The following subsystems are mutually exclusive
	// with regard to overall robot function cannot be considered
	// at the same time. Rather than create a complex selector
	// and state exclusions we will just prevent creation.
    /// WARNING WARNING WARNING: ONLY ONE

	
    public static OI oi;
	
	public static LightingControl lightingControl;	
	public static NavxIMU imu;
		
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = OI.instance();
		
		// Always instantiate the subsystems for this robot
		// Auto subsystem is constructed last as it may want to reference the others
		// (Not a best practice)
		driveSubsystem = new DriveSubsystem();
		intakeSubsystem = new IntakeSubsystem();
		visionSubsystem = new VisionSubsystem();
		//rampSubsystem = new RampSubsystem();		
		//elevatorSubsystem = new ElevatorSubsystem();
				
		imu = new NavxIMU();
		lightingControl = new LightingControl();
		
		autonomousSubsystem = new AutonomousSubsystem();
		autonomousSubsystem.initialize();
				
		// Add all subsystems for debugging
		addSubsystemToDebug(driveSubsystem);
        //addSubsystemToDebug(intakeSubsystem);
        addSubsystemToDebug(visionSubsystem);
        addSubsystemToDebug(autonomousSubsystem);
        //addSubsystemToDebug(elevatorSubsystem);
		showDebugInfo();		
		
        CameraServer.getInstance().startAutomaticCapture();
	}
	
	private void setSubsystemsDebug() 
	{
		driveSubsystem.setDiagnosticsFlag(true);
		intakeSubsystem.setDiagnosticsFlag(true);
		
        /// WARNING WARNING WARNING: ONLY ONE		
		//springShooterSubsystem.setDiagnosticsFlag(true);
		elevatorSubsystem.setDiagnosticsFlag(true);
		
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
	public void disabledPeriodic() {
		runWatch.start();
		Scheduler.getInstance().run();
		runWatch.stop();
	}
	
	Waypoint[] points = new Waypoint[]
			{
				new Waypoint(0, 0, 0),
				new Waypoint(1, 2, Pathfinder.d2r(45)),
                new Waypoint(3, 4, 0),
                new Waypoint(5,6, Pathfinder.d2r(45))
			};
	
	@Override
	public void autonomousInit() {
		autonomousSubsystem.convertGameData();
		
		runMode = RunMode.AUTO;
		oi.setAutoMode();
	}
	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		runWatch.start();
		Scheduler.getInstance().run();
		runWatch.stop();
	}
	@Override
	public void teleopInit() {
		runMode = RunMode.TELEOP;
		oi.setTeleopMode();
		
	}
	
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {	
		runWatch.start();
		Scheduler.getInstance().run();	
		runWatch.stop();
	}
	
	@Override
	public void testInit() {
		runMode = RunMode.TEST;

		setSubsystemsDebug();
		LiveWindow.setEnabled(false);
		
	}
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		runWatch.start();
		Scheduler.getInstance().run();	
		runWatch.stop();

	}
	
	// Called periodically all the time (regardless of mode)
	@Override
	public void robotPeriodic() {
		SmartDashboard.putString("CurrMode", runMode.name());
		
		loopWatch.stop();
		loopWatch.start();
		
		periodicSDdebugLoop.update();
	}
	
	// Some ancillary debugging stuff below here
	
	private Stopwatch runWatch = 
			new Stopwatch( "Run", 
			(name, max, min, avg) -> SmartDashboard.putNumber( "MaxRun", max) );
	private Stopwatch loopWatch = 
			new Stopwatch( "Loop", 
			(name, max, min, avg) -> SmartDashboard.putNumber( "MaxLoop", max) );
	private DoEveryN periodicSDdebugLoop = 
			new DoEveryN( 10, () -> putPeriodicSDdebug());
	
	
	private void putPeriodicSDdebug() {
		
//		SmartDashboard.putString( "IMU_Yaw", 
//				String.format("%.1f", imu.getYawDeg()));
//		SmartDashboard.putString( "IMU_Yawrate", 
//				String.format("%.1f", imu.getYawRateDps()));
//		SmartDashboard.putString( "Left_Position", 
//				String.format("%.1f", driveSubsystem.getLeftPosition_inch()));
//		SmartDashboard.putString( "Right_Position", 
//				String.format("%.1f", driveSubsystem.getRightPosition_inch()));
//		SmartDashboard.putString("Fwd_Velocity",
//				String.format("%.1f", driveSubsystem.getFwdVelocity_ips()));
//		SmartDashboard.putString( "Fwd_Current", 
//				String.format( "%.1f", driveSubsystem.getFwdCurrent()));
//		SmartDashboard.putString( "VisGearYaw",
//				String.format("%.1f", visionSubsystem.getGearAngle_deg()));
//		SmartDashboard.putString( "VisGearDist", 
//				String.format("%.1f", visionSubsystem.getGearDistance_inch()));
	}	
	private Set<Subsystem> subSystems = new HashSet<>();
	// Add Subsystem to the test set
	public void addSubsystemToDebug(Subsystem subsys) {
		subSystems.add(subsys);
	}
	
	// Put debug info on SmartDashboard
	private void showDebugInfo() 
	{
		// Show the Scheduler
		SmartDashboard.putData(Scheduler.getInstance());
		// Show the Subsystems
		for (Subsystem subsys : subSystems)
			SmartDashboard.putData(subsys);
	}	
}