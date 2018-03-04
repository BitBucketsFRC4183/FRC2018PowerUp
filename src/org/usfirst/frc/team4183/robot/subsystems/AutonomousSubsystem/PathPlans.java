package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.RobotTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

/*
 * PathPlans contains our pre-planned paths for autonomous operations
 * 
 * Each plan is converted into a trajectory at construction/initialization
 * so that it is ready for execution via Motion Profile processing.
 */
public class PathPlans 
{
	// Jaci Brunning's Pathfinder code starts with a trajectory configuration tool
	// that defines how paths will be converted to trajectories.
	//
	// We use this configuration to limit the max speed, acceleration, and jerk
	// we are willing to accept for a trajectory
	//
	// NOTE: All of Jaci's code assumes metric units (meters (m), m/s, m/s^2, m/s^3 etc)
	// This will be important for converting trajectories into motion profiles

	/// TODO: Need to consider cross over restrictions and choose... maybe options on scoring goal
	public enum PathPlanChoice 
	{
		// This enumeration corresponds to the similarly named Path and Trajectory
		NONE,
		TEST0,
		autoTest
	}
	
	private static SendableChooser<PathPlanChoice> pathChooser;
	
	PathPlans()
	{
		pathChooser = new SendableChooser<PathPlanChoice>();
		pathChooser.addDefault( "NONE",		PathPlanChoice.NONE);
		pathChooser.addObject(  "TEST-0",   PathPlanChoice.TEST0);
		pathChooser.addObject("autoTest", PathPlanChoice.autoTest);
		SmartDashboard.putData( "Path Plan", pathChooser);
		
	}

	static Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, // Type of curve to fit
													 Trajectory.Config.SAMPLES_LOW,     // Smooth fit (high) or fast fit (low)
													 RobotMap.MOTION_PROFILE_PERIOD_MS / 1000.0, // Time between segments
													 0.3048*6, 	    // Max speed m/s
													 2.0, 			// Max acceleration m/s^2
													 60.0);			// Max jerk m/s^3
	
	// Paths are defined as waypoints in x,y,heading
	//
	// heading is defined to be 0 parallel to the x-axis
	//
	// In this example, define the path start as 0,0,0 meaning we are aligned to the x-axis
	//	
	
	private final static double R_m = 0.5; // Test radius in meters (so 1.5 m is approx. 5 feet)
    private static Waypoint[] testPath0 = new Waypoint[] 
    {
    		// A simple S curve to reach point across a square
            new Waypoint(0, 		0,      Pathfinder.d2r(0)),
            new Waypoint(R_m, 		0,      Pathfinder.d2r(0)),
            new Waypoint(2*R_m, 	R_m, 	Pathfinder.d2r(90)),
            new Waypoint(2*R_m, 	3*R_m, 	Pathfinder.d2r(90)),
            new Waypoint(3*R_m, 	4*R_m, 	Pathfinder.d2r(0)),
            new Waypoint(4*R_m, 	4*R_m, 	Pathfinder.d2r(0))
    };
    
    private static Waypoint[] autoTest = new Waypoint[]
    {
    		new Waypoint(0, 0, Pathfinder.d2r(0)),
    		new Waypoint(0.864, -0.699, Pathfinder.d2r(-45)),
    		new Waypoint(1.817+0.5, -1.245, Pathfinder.d2r(0)),
    		//ALT1:
    		//new Waypoint(6.008, 1.245, Pathfinder.d2r(0)),
    		//new Waypoint(6.884, 0.689, Pathfinder.d2r(-45)),
    		//new Waypoint(7.367, -0.124, Pathfinder.d2r(-90))
    		//ALT 2:
    		new Waypoint(2.922+0.3, -1.245+0.3, Pathfinder.d2r(45)),
    		new Waypoint(3.405, 0, Pathfinder.d2r(90))
	    		
    };
   
    public static RobotTrajectory testTrajectory0;
    
    public static RobotTrajectory autoTestTrajectory;

    public static void initialize()
    {
    
    	testTrajectory0 = new RobotTrajectory("Test0");//determines the path
	    testTrajectory0.center = Pathfinder.generate(autoTest, config);
	
	    // We don't need to store the modifier persistently
	    TankModifier modifier = new TankModifier(testTrajectory0.center).modify(RobotMap.inch2Meter(RobotMap.WHEEL_TRACK_INCHES));
	
	    // Extract the right and left trajectories
	    testTrajectory0.left = modifier.getLeftTrajectory();
	    testTrajectory0.right = modifier.getRightTrajectory();
	 }
    
	public static RobotTrajectory getSelectedTrajectory() 
	{
		RobotTrajectory trajectory = null;
		
		switch (pathChooser.getSelected())
		{
		case NONE:
			break;
		case TEST0:
			//System.out.println("TEST0 PATH SELECTED");			
			trajectory = PathPlans.testTrajectory0;			
			break;
		case autoTest:
		default:
			//System.out.println("BAD PATH CHOICE");
			break;
		
		}
		
		return trajectory;
		
	}

}
