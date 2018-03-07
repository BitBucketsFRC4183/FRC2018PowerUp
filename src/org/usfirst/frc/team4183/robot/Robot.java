/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc.team4183.robot;
import java.util.HashSet;
import java.util.Set;
import org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem.AutonomousSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.DriveSubsystem.DriveSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.ElevatorSubsystem.ElevatorSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.RampSubsystem.RampSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.VisionSubsystem.VisionSubsystem;
import org.usfirst.frc.team4183.robot.subsystems.SubsystemUtilities.DiagnosticsInformation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4183.utils.DoEveryN;
import org.usfirst.frc.team4183.utils.Stopwatch;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

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
	public static VisionSubsystem visionSubsystem;
	public static RampSubsystem rampSubsystem;
	public static ElevatorSubsystem elevatorSubsystem;

	public static AutonomousSubsystem autonomousSubsystem;
	
	public static OI oi;
	
	// Here for now, but may not be used this year
	public static LightingControl lightingControl;	
	public static NavxIMU imu;
	
	public static SendableChooser<DiagnosticsInformation> diagInformation;
		
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
		rampSubsystem = new RampSubsystem();		
		elevatorSubsystem = new ElevatorSubsystem();
				
		imu = new NavxIMU();
		lightingControl = new LightingControl();
		
		autonomousSubsystem = new AutonomousSubsystem();
		autonomousSubsystem.initialize();
				
		diagInformation = new SendableChooser<DiagnosticsInformation>();
		diagInformation.addDefault("Subsystem_Basic", DiagnosticsInformation.SUBSYSTEM_BASIC);
		diagInformation.addObject("Subsystem_Extended", DiagnosticsInformation.SUBSYSTEM_EXTENDED);
		
		SmartDashboard.putData("DiagInfo", diagInformation);
		
		// Add all subsystems for debugging
		addSubsystemToDebug(driveSubsystem);
        addSubsystemToDebug(intakeSubsystem);
        addSubsystemToDebug(visionSubsystem);
        addSubsystemToDebug(elevatorSubsystem);
        addSubsystemToDebug(rampSubsystem);

        addSubsystemToDebug(autonomousSubsystem);
        
        showDebugInfo();		
		
        /// TODO: Consider moving to vision subsystem
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(640, 480);
        camera.setFPS(120);
	}
	
	private void setSubsystemsDebug() 
	{
		driveSubsystem.setDiagnosticsFlag(true);
		intakeSubsystem.setDiagnosticsFlag(true);		
		elevatorSubsystem.setDiagnosticsFlag(true);
		
	}
	
	protected void initializePhysicalSubsystems()
	{
		// Only the physical subsystems
		driveSubsystem.initialize();
		intakeSubsystem.initialize();
		elevatorSubsystem.initialize();
		
		/// TODO: climbing and or ramps
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
		
		// NOTE: We don't use the default commands because it causes problems
		// when trying to cross initiate tasks from an autonomous state machine
		// Instead we explicitly initialize the subsystems to start the
		// the first commands states
		initializePhysicalSubsystems();

	}
	@Override
	public void disabledPeriodic() {
		runWatch.start();
		Scheduler.getInstance().run();
		runWatch.stop();
	}
	
	@Override
	public void autonomousInit() {
		
		runMode = RunMode.AUTO;
		oi.setAutoMode();
		
		initializePhysicalSubsystems();
		
		autonomousSubsystem.start();

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
		
		initializePhysicalSubsystems();
		
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
		SmartDashboard.putBoolean("ElevatorBottom", Robot.oi.btnElevatorBottom.get());
		
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