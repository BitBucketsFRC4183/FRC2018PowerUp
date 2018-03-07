package org.usfirst.frc.team4183.robot.subsystems.AutonomousSubsystem;

import org.usfirst.frc.team4183.robot.Robot;
import org.usfirst.frc.team4183.robot.RobotMap;
import org.usfirst.frc.team4183.utils.Positions.StartingPosition;
import org.usfirst.frc.team4183.utils.Positions;
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
		// NOTE: The test path plan is selected directly when needed
		RIGHT_START_RIGHT_SWITCH,
		RIGHT_START_LEFT_SWITCH,
		CENTER_START_RIGHT_SWITCH
	}
	
	private static SendableChooser<PathPlanChoice> pathChooser;
	
	private static SendableChooser<StartingPosition> startingPositionChooser;
	
	public enum PrimaryRole
	{
		SWITCH,
		SCALE,
		EXCHANGE,
		CROSS_LINE
	}
	
	private static SendableChooser<PrimaryRole> primaryRollChooser;
	
	public enum CrossingMode
	{
		ENABLE_CROSSING,
		DISABLE_CROSSING
	}
	
	private static SendableChooser<CrossingMode> crossingModeChooser;
	
	
	PathPlans()
	{
		
	}

	// For now, a single configuration is sufficient
	// If we really need different ones then we will make them
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
	
	private final static double R_m = 1.0; // Test radius in meters
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
    
    // **************************************
    // NOTE: NOTE: NOTE: NOTE:
    // **************************************
    // The values for the paths contain arithmetic to keep track of where we started
    // and may be replaced with final values after testing
    
    private static Waypoint[] centerStartRightSwitchPath = new Waypoint[]
    {
    		new Waypoint(0,            0,      Pathfinder.d2r(0)),
    		new Waypoint(.8382,        -.6096, Pathfinder.d2r(-40)),
    		new Waypoint(2.4638 + 0.15,-1.0668,Pathfinder.d2r(0))
    };
    
    private static Waypoint[] centerStartLeftSwitchPath = new Waypoint[]
    {
    		new Waypoint(0,0, Pathfinder.d2r(0)),
    		new Waypoint(.8382, .6096 + 0.3, Pathfinder.d2r(40)),
    		new Waypoint(2.4638+.15, 1.0668+0.3, Pathfinder.d2r(0))
    };
    
    private static Waypoint[] rightStartRightSwitchPath = new Waypoint[]
    {
    		new Waypoint(0,                     0,     Pathfinder.d2r(0)),
    		new Waypoint(0.864,     -0.699-0.3048,     Pathfinder.d2r(-45)),
    		new Waypoint(1.817+0.5, -1.245-0.3048,     Pathfinder.d2r(0)),
    		new Waypoint(2.922+0.3, -1.245+0.3-0.3048, Pathfinder.d2r(45)),
    		new Waypoint(3.405,          0-0.3048,     Pathfinder.d2r(100))
	    		
    };
    
    // basically a left/right (y) mirror of the above
    // Rather than loading programmatically, we place the value explicitly
    // just in case we need to make slight adjustments
    private static Waypoint[] leftStartLeftSwitchPath = new Waypoint[]
    {
    		new Waypoint(0,         0,                Pathfinder.d2r(0)),
    		new Waypoint(0.864,     0.699-0.3048,     Pathfinder.d2r(45)),
    		new Waypoint(1.817+0.5, 1.245-0.3048,     Pathfinder.d2r(0)),
    		new Waypoint(2.922+0.3, 1.245-0.3-0.3048,     Pathfinder.d2r(-45)),
    		new Waypoint(3.405,         0,     Pathfinder.d2r(-100))
    };

    private static Waypoint[] rightStartLeftSwitchPath = new Waypoint[]
    {
    		new Waypoint(0,              0, Pathfinder.d2r(0)),
    		new Waypoint(0.864,     -0.699, Pathfinder.d2r(-45)),
    		new Waypoint(1.817+0.5, -1.245, Pathfinder.d2r(0)),
    		
    		new Waypoint(4.318,       -1.245-0.3048, Pathfinder.d2r(0)),
    		new Waypoint(5.182,       0.254-0.3048,  Pathfinder.d2r(90)),
    		new Waypoint(5.182,       3.937-0.3048,  Pathfinder.d2r(90)),
    		new Waypoint(4.318+0.3048,   5.436-3*0.3048,  Pathfinder.d2r(180)),
    		new Waypoint(3.600+0.3048,   5.136-3*0.3048,  Pathfinder.d2r(225)),
    		new Waypoint(3.405+0.3048,   4.191-3*0.3048,  Pathfinder.d2r(310))
	    		
    };

    // Again, a mirror but nor programmatically just in case
    private static Waypoint[] leftStartRightSwitchPath = new Waypoint[]
    {
    		new Waypoint(0,              0, Pathfinder.d2r(0)),
    		new Waypoint(0.864,     0.699, Pathfinder.d2r(45)),
    		new Waypoint(1.817+0.5, 1.245, Pathfinder.d2r(0)),
    		
    		new Waypoint(4.318,       1.245-0.3048, Pathfinder.d2r(0)),
    		new Waypoint(5.182,       -0.254-0.3048,  Pathfinder.d2r(-90)),
    		new Waypoint(5.182,       -3.937-0.3048,  Pathfinder.d2r(-90)),
    		new Waypoint(4.318+0.3048,  -5.436-0.3048,  Pathfinder.d2r(-180)),
    		new Waypoint(3.600+0.3048,  -5.136+2*0.3048,  Pathfinder.d2r(-225)),
    		new Waypoint(3.405+0.3048,  -4.191+2*0.3048,  Pathfinder.d2r(-310))
	    		
    };
    
    public static RobotTrajectory testTrajectory0;
    
    public static RobotTrajectory rightStartRightSwitchTrajectory;
    public static RobotTrajectory leftStartLeftSwitchTrajectory;
    
    public static RobotTrajectory centerStartRightSwitchTrajectory;
    public static RobotTrajectory centerStartLeftSwitchTrajectory;
    
    public static RobotTrajectory rightStartLeftSwitchTrajectory;
    public static RobotTrajectory leftStartRightSwitchTrajectory;
    
    

    public static void initialize()
    {
    	startingPositionChooser = new SendableChooser<StartingPosition>();
    	startingPositionChooser.addDefault(  "LEFT",    StartingPosition.LEFT);
    	startingPositionChooser.addObject(   "CENTER",  StartingPosition.CENTER);
    	startingPositionChooser.addObject(   "RIGHT",    StartingPosition.RIGHT);
    	SmartDashboard.putData( "Starting Position", startingPositionChooser);
    	
    	
    	primaryRollChooser = new SendableChooser<PrimaryRole>();
    	primaryRollChooser.addDefault(  "CROSS LINE",    PrimaryRole.CROSS_LINE);
    	primaryRollChooser.addObject(   "SWITCH",        PrimaryRole.SWITCH);
    	primaryRollChooser.addObject(   "SCALE",         PrimaryRole.SCALE);
    	primaryRollChooser.addObject(   "EXCAHNGE",      PrimaryRole.EXCHANGE);
    	SmartDashboard.putData( "Primary Roll", primaryRollChooser);
    	
    	crossingModeChooser = new SendableChooser<CrossingMode>() ;
    	crossingModeChooser.addDefault(  "DISABLE CROSSING",    CrossingMode.DISABLE_CROSSING);
    	crossingModeChooser.addObject(   "ENABLE CROSSING",     CrossingMode.ENABLE_CROSSING);
    	SmartDashboard.putData( "Crossing Mode", crossingModeChooser);
    	
    	/// TODO: This will be replaced with logic based on the above choices
		pathChooser = new SendableChooser<PathPlanChoice>();
		pathChooser.addDefault( "NONE",		PathPlanChoice.NONE);
		pathChooser.addObject("RIGHT RIGHT", PathPlanChoice.RIGHT_START_RIGHT_SWITCH);
		pathChooser.addObject("RIGHT LEFT", PathPlanChoice.RIGHT_START_LEFT_SWITCH);
		pathChooser.addDefault("CENTER RIGHT",PathPlanChoice.CENTER_START_RIGHT_SWITCH);
		SmartDashboard.putData( "Path Plan", pathChooser);
		
    	/// TODO: May want to pre-compute these and store them as files to speed up startup
    	
	    
	  //***********
    	testTrajectory0 = new RobotTrajectory("Test0");//determines the path
	    testTrajectory0.center = Pathfinder.generate(testPath0, config);
	    
	    // We don't need to store the modifier persistently
	    TankModifier modifier = new TankModifier(testTrajectory0.center).modify(RobotMap.inch2Meter(RobotMap.WHEEL_TRACK_INCHES));
	
	    // Extract the right and left trajectories
	    testTrajectory0.left = modifier.getLeftTrajectory();
	    testTrajectory0.right = modifier.getRightTrajectory();
	    
	    
	 //***********
	    rightStartRightSwitchTrajectory = new RobotTrajectory("rightStartRightSwitch");
	    rightStartRightSwitchTrajectory.center = Pathfinder.generate(rightStartRightSwitchPath, config);

	    // We don't need to store the modifier persistently
	    modifier = new TankModifier(rightStartRightSwitchTrajectory.center).modify(RobotMap.inch2Meter(RobotMap.WHEEL_TRACK_INCHES));
	    
	    // Extract the right and left trajectories
	    rightStartRightSwitchTrajectory.left = modifier.getLeftTrajectory();
	    rightStartRightSwitchTrajectory.right = modifier.getRightTrajectory();

	//***********
	    leftStartLeftSwitchTrajectory = new RobotTrajectory("leftStartLeftSwitch");
	    leftStartLeftSwitchTrajectory.center = Pathfinder.generate(leftStartLeftSwitchPath, config);

	    // We don't need to store the modifier persistently
	    modifier = new TankModifier(leftStartLeftSwitchTrajectory.center).modify(RobotMap.inch2Meter(RobotMap.WHEEL_TRACK_INCHES));
	    
	    // Extract the right and left trajectories
	    leftStartLeftSwitchTrajectory.left = modifier.getLeftTrajectory();
	    leftStartLeftSwitchTrajectory.right = modifier.getRightTrajectory();
	    
	    
	  //***********
	    rightStartLeftSwitchTrajectory = new RobotTrajectory("rightStartLeftSwitch");
	    rightStartLeftSwitchTrajectory.center = Pathfinder.generate(rightStartLeftSwitchPath, config);

	    // We don't need to store the modifier persistently
	    modifier = new TankModifier(rightStartLeftSwitchTrajectory.center).modify(RobotMap.inch2Meter(RobotMap.WHEEL_TRACK_INCHES));

	    
	    // Extract the right and left trajectories
	    rightStartLeftSwitchTrajectory.left = modifier.getLeftTrajectory();
	    rightStartLeftSwitchTrajectory.right = modifier.getRightTrajectory();

	 //***********
	    leftStartRightSwitchTrajectory = new RobotTrajectory("leftStartRightSwitch");
	    leftStartRightSwitchTrajectory.center = Pathfinder.generate(leftStartRightSwitchPath, config);

	    // We don't need to store the modifier persistently
	    modifier = new TankModifier(leftStartRightSwitchTrajectory.center).modify(RobotMap.inch2Meter(RobotMap.WHEEL_TRACK_INCHES));

	    
	    // Extract the right and left trajectories
	    leftStartRightSwitchTrajectory.left = modifier.getLeftTrajectory();
	    leftStartRightSwitchTrajectory.right = modifier.getRightTrajectory();
	    
	  //***********
	    centerStartRightSwitchTrajectory = new RobotTrajectory("centerStartRightSwitch");
	    centerStartRightSwitchTrajectory.center = Pathfinder.generate(centerStartRightSwitchPath, config);

	    modifier = new TankModifier(centerStartRightSwitchTrajectory.center).modify(RobotMap.inch2Meter(RobotMap.WHEEL_TRACK_INCHES));
		
	    centerStartRightSwitchTrajectory.left = modifier.getLeftTrajectory();
	    centerStartRightSwitchTrajectory.right = modifier.getRightTrajectory();
	    
	    //***********
	    centerStartLeftSwitchTrajectory = new RobotTrajectory("centerStartLeftSwitch");
	    centerStartLeftSwitchTrajectory.center = Pathfinder.generate(centerStartLeftSwitchPath, config);

	    modifier = new TankModifier(centerStartLeftSwitchTrajectory.center).modify(RobotMap.inch2Meter(RobotMap.WHEEL_TRACK_INCHES));
		
	    centerStartLeftSwitchTrajectory.left = modifier.getLeftTrajectory();
	    centerStartLeftSwitchTrajectory.right = modifier.getRightTrajectory();
	    
	    
	 }
    
	public static RobotTrajectory getSelectedTrajectory() 
	{
		RobotTrajectory trajectory = null;
		
		/// TODO: Replace below with logic based on better selectors
		/// including starting side, roll, crossing allowance
		
		// Description as comments
		//		Available Primary choices:
		//			Switch
		//			Scale
		//          Exchange
		//			Cross Line
		//		Decisions for Switch and Scale:
		//			If primary is on same side, proceed
		//			If primary is on opposite side and crossing allowed, proceed
		//			If primary is on opposite side and crossing DISALLOWED, check for secondary
		//			If secondary is on same side, proceed
		//			Secondary is on opposite side and crossing DISALLOWED so cross line and wait
		//      Decision for Exchange:
		//			If position is center, proceed
		//			If position is right or left, cross line and wait
		//
		/// TODO: Consider exchange as choice?
		
		/// TODO: Again, replace the below with something like above
		
		
		//INSERT SECONDARY GOALS WHEN THE SELECTOR IS ADDED
		CrossingMode crossingMode = crossingModeChooser.getSelected();
		
		Positions.GenericPositions switchPos = Robot.autonomousSubsystem.getSwitchPosition();
		Positions.GenericPositions scalePos = Robot.autonomousSubsystem.getScalePosition();
		
		switch (primaryRollChooser.getSelected())
		{
		case SWITCH:
			if (startingPositionChooser.getSelected() == StartingPosition.LEFT)
			{
				if (switchPos == Positions.GenericPositions.LEFT)
				{
					System.out.println("LEFT START LEFT SWITCH (PRIMARY)");
					trajectory = PathPlans.leftStartLeftSwitchTrajectory;
				}
				// explicitly check for right in case FMS has bug
				else if (switchPos == Positions.GenericPositions.RIGHT && crossingMode == CrossingMode.ENABLE_CROSSING)
				{
					System.out.println("LEFT START RIGHT SWITCH (PRIMARY)");
					trajectory = PathPlans.leftStartRightSwitchTrajectory;
				}
				else // Primary is out of reach or unspecified
				{
					if (scalePos == Positions.GenericPositions.LEFT)
					{
						System.out.println("LEFT START RIGHT SCALE (SECONDARY)");
						/// TODO: trajectory = PathPlans.leftStartLeftScaleTrajectory;
					}
					else
					{
						System.out.println("NO SOLUTION: DRIVING FORWARD - CROSSING LINE");
					}
					// In all other cases crossing is disabled or FMS messed up, so just drive forward
				}
			}
			else if (startingPositionChooser.getSelected() == StartingPosition.CENTER)
			{
				if (switchPos == Positions.GenericPositions.LEFT)
				{
					System.out.println("CENTER START LEFT SWITCH (PRIMARY)");
					trajectory = PathPlans.centerStartLeftSwitchTrajectory;
					
				}
				else if (switchPos == Positions.GenericPositions.RIGHT)
				{
					System.out.println("CENTER START RIGHT SWITCH (PRIMARY)");
					trajectory = PathPlans.centerStartRightSwitchTrajectory;
				}
				else
				{
					System.out.println("NO SOLUTION: FMS ERROR");
				}
				
				// NOTE: If FMS messes up, we will drive into stack... do we want to suppress?
			}
			else if (startingPositionChooser.getSelected() == StartingPosition.RIGHT)
			{
				if (switchPos == Positions.GenericPositions.RIGHT)
				{
					trajectory = PathPlans.rightStartRightSwitchTrajectory;
					System.out.println("RIGHT START RIGHT SWITCH (PRIMARY)");	
				}
				else if (switchPos == Positions.GenericPositions.LEFT && crossingMode == CrossingMode.ENABLE_CROSSING)
				{
					System.out.println("RIGHT START LEFT SWITCH (PRIMARY)");
					trajectory = PathPlans.rightStartLeftSwitchTrajectory;
				}
				else
				{
					if (scalePos == Positions.GenericPositions.RIGHT)
					{
						System.out.println("RIGHT START RIGHT SCALE (SECONDARY)");
						/// TODO: trajectory = PathPlans.rightStartRightScaleTrajectory;
					}
					else
					{
						System.out.println("DRIVING FORWARD - CROSSING LINE");
					}
					// In all other cases crossing is disabled or FMS messed up, so just drive forward
				}
			}
			break;
			
		case SCALE:
			if (startingPositionChooser.getSelected() == StartingPosition.LEFT)
			{
				if (scalePos == Positions.GenericPositions.LEFT)
				{
					System.out.println("LEFT START LEFT SCALE (PRIMARY)");
					/// TODO: trajectory = PathPlans.leftStartLeftScaleTrajectory;
				}
				// explicitly check for right in case FMS has bug
				else if (scalePos == Positions.GenericPositions.RIGHT && crossingMode == CrossingMode.ENABLE_CROSSING)
				{
					System.out.println("LEFT START RIGHT SCALE (PRIMARY)");
					/// TODO: trajectory = PathPlans.leftStartRightScaleTrajectory;
				}
				else // Primary is out of reach or unspecified
				{
					if (switchPos == Positions.GenericPositions.LEFT)
					{
						System.out.println("LEFT START LEFT SWITCH (SECONDARY)");
						trajectory = PathPlans.leftStartLeftSwitchTrajectory;
					}
					else
					{
						System.out.println("DRIVING FORWARD - CROSSING LINE");
					}
					// In all other cases crossing is disabled or FMS messed up, so just drive forward
				}				
			}
			else if (startingPositionChooser.getSelected() == StartingPosition.RIGHT)
			{
				if (scalePos == Positions.GenericPositions.RIGHT)
				{
					System.out.println("RIGHT START RIGHT SCALE (PRIMARY)");
					/// TODO: trajectory = PathPlans.rightStartRightScaleTrajectory;
				}
				// explicitly check for right in case FMS has bug
				else if (scalePos == Positions.GenericPositions.LEFT && crossingMode == CrossingMode.ENABLE_CROSSING)
				{
					System.out.println("RIGHT START LEFT SCALE (PRIMARY)");
					/// TODO: trajectory = PathPlans.rightStartLeftScaleTrajectory;
				}
				else // Primary is out of reach or unspecified
				{
					if (switchPos == Positions.GenericPositions.RIGHT)
					{
						System.out.println("RIGHT START RIGHT SWITCH (SECONDARY)");
						trajectory = PathPlans.rightStartRightSwitchTrajectory;
					}
					else
					{
						System.out.println("DRIVING FORWARD - CROSSING LINE");
					}
					
					// In all other cases crossing is disabled or FMS messed up, so just drive forward
				}				
				
			}
			break;
			
		case EXCHANGE:
			if (startingPositionChooser.getSelected() == StartingPosition.CENTER)
			{
				/// TODO:
			}
			else
			{
				System.out.println("DRIVER POSITION ERROR: Can only do exchange from center");
			}
			break;
			
		case CROSS_LINE: //Done purposely
		default:
			System.out.println("RETURNING DRIVE FORWARD");
			break;
		}
		
		/*
		switch (pathChooser.getSelected())
		{
		case NONE:
			break;
		case RIGHT_START_RIGHT_SWITCH:
			System.out.println("RIGHT_START_RIGHT_SWITCH PATH SELECTED");			
			trajectory = PathPlans.rightStartRightSwitchTrajectory;
			break;
		case RIGHT_START_LEFT_SWITCH:
			System.out.println("RIGHT_START_LEFT_SWITCH PATH SELECTED");			
			trajectory = PathPlans.rightStartLeftSwitchTrajectory;
			break;
		case CENTER_START_RIGHT_SWITCH:
			System.out.println("CENTER_START_RIGHT_SWITCH PATH SELECTED");
			trajectory= PathPlans.centerStartRightSwitchTrajectory;
			break;
		default:
			System.out.println("BAD PATH CHOICE");
			break;
		
		}
	*/	
		return trajectory;
		
	}

}
