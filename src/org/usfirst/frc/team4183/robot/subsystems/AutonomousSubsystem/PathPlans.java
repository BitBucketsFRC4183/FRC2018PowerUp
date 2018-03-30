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
import java.io.*;

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
	static Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, // Type of curve to fit
													 Trajectory.Config.SAMPLES_LOW,     // Smooth fit (high) or fast fit (low)
													 RobotMap.MOTION_PROFILE_PERIOD_MS / 1000.0, // Time between segments
													 0.3048*6, 	    // Max speed m/s
													 2.0, 			// Max acceleration m/s^2
													 60.0);			// Max jerk m/s^3
	
	//used specifically for the LLSCALE
	static Trajectory.Config config2 = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, // Type of curve to fit
			 Trajectory.Config.SAMPLES_LOW,     // Smooth fit (high) or fast fit (low)
			 RobotMap.MOTION_PROFILE_PERIOD_MS / 1000.0, // Time between segments
			 0.3048*4, 	    // Max speed m/s
			 2.0, 			// Max acceleration m/s^2
			 40.0);			// Max jerk m/s^3
	
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
    
    private static Waypoint[] rightStartMoveOnlyPath = new Waypoint[]
    {
		new Waypoint(0,                     0,     Pathfinder.d2r(0)),
		new Waypoint(0.864,     -0.699-0.3048,     Pathfinder.d2r(-45)),
		new Waypoint(2.317+.6,     -1.245-0.3048,     Pathfinder.d2r(0)),	    		
    };
    
    private static Waypoint[] leftStartMoveOnlyPath = new Waypoint[]
    {
    		new Waypoint(0,         0,                Pathfinder.d2r(0)),
    		new Waypoint(0.864,     0.699-0.3048,     Pathfinder.d2r(45)),
    		new Waypoint(2.317+0.6,     1.245-0.3048,     Pathfinder.d2r(0)),
    		//Adjust the final position of this
    };
    
    // *****************************************************************
    // SWITCH PATHS (Modifications shown as explicit arithmetic)
    // *****************************************************************
    private static Waypoint[] centerStartRightSwitchPath = new Waypoint[]
    {
    		new Waypoint(0,            0,       Pathfinder.d2r(0)),
    		new Waypoint(0.8382,       -0.6096, Pathfinder.d2r(-40)),
    		new Waypoint(2.6138+(0.5*0.3048),      -1.0668,  Pathfinder.d2r(0))
    };
    
    private static Waypoint[] centerStartLeftSwitchPath = new Waypoint[]
    {
    		new Waypoint(0,              0, Pathfinder.d2r(0)),
    		new Waypoint(0.8382,    0.9096, Pathfinder.d2r(40)),	// Includes y-bias
    		new Waypoint(2.6138+.3,    1.3668, Pathfinder.d2r(0))		// Includes y-bias
    };
    
    private static Waypoint[] rightStartRightSwitchPath = new Waypoint[]
    {
    		new Waypoint(0,                     0,     Pathfinder.d2r(0)),
    		new Waypoint(0.864,     -0.699-0.3048,     Pathfinder.d2r(-45)),
    		new Waypoint(2.317,     -1.245-0.3048,     Pathfinder.d2r(0)),
    		
    		new Waypoint(3.222, -1.245+0.3-0.3048, 	   Pathfinder.d2r(45)),
    		new Waypoint(3.405,          0,     Pathfinder.d2r(100))
	    		
    };
    
    // basically a left/right (y) mirror of the above
    // Rather than loading programmatically, we place the value explicitly
    // just in case we need to make slight adjustments
    private static Waypoint[] leftStartLeftSwitchPath = new Waypoint[]
    {
    		new Waypoint(0,         0,                Pathfinder.d2r(0)),
    		new Waypoint(0.864,     0.699-0.3048,     Pathfinder.d2r(45)),
    		new Waypoint(2.317,     1.245-0.3048,     Pathfinder.d2r(0)),
    		
    		new Waypoint(3.222,     1.245-0.3-0.3048, Pathfinder.d2r(-45)),
    		new Waypoint(3.405,         0-0.3048,            Pathfinder.d2r(-100))
    };

    private static Waypoint[] rightStartLeftSwitchPath = new Waypoint[]
    {
    		new Waypoint(0,              0,        		Pathfinder.d2r(0)),
    		new Waypoint(0.864,     -0.699-0.3048, 		Pathfinder.d2r(-45)),
    		new Waypoint(2.317,     -1.245-0.3048, 		Pathfinder.d2r(0)),
    		
    		new Waypoint(4.318,         -1.245-0.3048, 		Pathfinder.d2r(0)),
    		new Waypoint(5.182,          0.254-0.3048,  	Pathfinder.d2r(90)),
    		new Waypoint(5.182,          3.937-0.3048,  	Pathfinder.d2r(90)),
    		new Waypoint(4.318+0.3048,   5.436-3*0.3048,  	Pathfinder.d2r(180)),
    		new Waypoint(3.600+0.3048,   5.136-3*0.3048,  	Pathfinder.d2r(225)),
    		new Waypoint(3.405+0.3048,   4.191-3*0.3048,  	Pathfinder.d2r(310))
	    		
    };
    
    // Again, a mirror but not programmatically just in case
    // NOTE: Includes y-biases observed
    private static Waypoint[] leftStartRightSwitchPath = new Waypoint[]
    {
    		new Waypoint(0,              0,               Pathfinder.d2r(0)),	// NOTE: y-bias
    		new Waypoint(0.864,      0.699,               Pathfinder.d2r(45)),
    		new Waypoint(1.817+0.5,  1.245,               Pathfinder.d2r(0)),
    		
    		new Waypoint(4.318,          1.245, 	      Pathfinder.d2r(0)),
    		new Waypoint(5.182,         -0.254,    		  Pathfinder.d2r(-90)),
    		new Waypoint(5.182,         -3.937,           Pathfinder.d2r(-90)),
    		new Waypoint(4.318+0.3048,  -5.436+2*0.3048,  Pathfinder.d2r(-180)),
    		new Waypoint(3.600+0.3048,  -5.136+2*0.3048,  Pathfinder.d2r(-225)),
    		new Waypoint(3.405+0.3048,  -4.191+2*0.3048,  Pathfinder.d2r(-310))
	    		
    };
    
    // ***************************************************************************
    // SCALE PATHS (Modifications shown as explicit arithmetic)
    // ***************************************************************************
    private static Waypoint[] rightStartRightScalePath = new Waypoint[]
    {
        new Waypoint(0,                     0,     Pathfinder.d2r(0)),
        new Waypoint(0.864,            -1.004,     Pathfinder.d2r(-45)),
        new Waypoint(2.317,            -1.550,     Pathfinder.d2r(0)),
        new Waypoint(5.508,            -1.550,     Pathfinder.d2r(0)),
        new Waypoint(6.682+(0.3048*0.75), -0.376+(0.3048*0.75), Pathfinder.d2r(45+15))
    };

    private static Waypoint[] leftStartLeftScalePath = new Waypoint[]
    {
        new Waypoint(0,                     0,                  Pathfinder.d2r(0)),
        new Waypoint(1.155/*0.864*/,             0.648/*1.3*//*1.004*//*-0.3048*2*/,     Pathfinder.d2r(25)),
        new Waypoint(2.706,             0.664/*-0.3048*2*/,     Pathfinder.d2r(-20)),
        new Waypoint(5.705,             -0.427/*-0.3048*3*/,     Pathfinder.d2r(-20)),
        new Waypoint(6.607,             -1.33/*-0.3048*2*/,     Pathfinder.d2r(-45))
    };
    
    private static Waypoint[] rightStartLeftScalePath = new Waypoint[]
    {
        new Waypoint(0,                     0, 		Pathfinder.d2r(0)),
        new Waypoint(0.864,            -1.004, 		Pathfinder.d2r(-45)),
        new Waypoint(2.317, 	       -1.550, 		Pathfinder.d2r(0)),

        new Waypoint(4.318,            -1.550, 		Pathfinder.d2r(0)),
        new Waypoint(5.182,       	   -0.051,  	Pathfinder.d2r(90)),
        new Waypoint(5.182,             3.632,  	Pathfinder.d2r(90)),
        new Waypoint(4.623,             4.522,   	Pathfinder.d2r(90)),

        new Waypoint(5.062,   	         5.088,  	Pathfinder.d2r(0)),
        new Waypoint(5.812,              5.088,  	Pathfinder.d2r(0)),
        new Waypoint(6.790,              4.110,  	Pathfinder.d2r(-45))

    };
    
    private static Waypoint[] leftStartRightScalePath = new Waypoint[]
    {
        new Waypoint(0,                     0, 		Pathfinder.d2r(0)),
        new Waypoint(0.864,             1.004, 		Pathfinder.d2r(45)),
        new Waypoint(2.317, 	        1.550, 		Pathfinder.d2r(0)),

        new Waypoint(4.318,             1.550, 		Pathfinder.d2r(0)),
        new Waypoint(5.182,       	    0.051,  	Pathfinder.d2r(-90)),
        new Waypoint(5.182,            -3.632,  	Pathfinder.d2r(-90)),
        new Waypoint(4.623,            -4.522,   	Pathfinder.d2r(-90)),

        new Waypoint(5.062,   	        -5.088,  	Pathfinder.d2r(0)),
        new Waypoint(5.812,             -5.088,  	Pathfinder.d2r(0)),
        new Waypoint(6.790,             -4.110,  	Pathfinder.d2r(45))

    };    
        
    public static RobotTrajectory testTrajectory0;
    
    // *************************** MOTION ONLY TRAJECTORIES ************************
    public static RobotTrajectory leftStartMoveOnlyTrajectory;
    public static RobotTrajectory rightStartMoveOnlyTrajectory;

    // *************************** SWITCH TRAJECTORIES ************************
    public static RobotTrajectory rightStartRightSwitchTrajectory;
    public static RobotTrajectory leftStartLeftSwitchTrajectory;
    
    public static RobotTrajectory centerStartRightSwitchTrajectory;
    public static RobotTrajectory centerStartLeftSwitchTrajectory;
    
    public static RobotTrajectory rightStartLeftSwitchTrajectory;
    public static RobotTrajectory leftStartRightSwitchTrajectory;
    
    // *************************** SCALE TRAJECTORIES ************************
    public static RobotTrajectory rightStartRightScaleTrajectory;
    public static RobotTrajectory leftStartLeftScaleTrajectory;
        
    public static RobotTrajectory rightStartLeftScaleTrajectory;
    public static RobotTrajectory leftStartRightScaleTrajectory;
    
    public static void processFiles() 
    {
    	// Simple toggle to write files for later use
    	if(SmartDashboard.getBoolean("Write Paths", false)==true)
    	{
    		SmartDashboard.putBoolean("Write Paths", false);
    		try {
				writeTrajectoriesToFiles();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				System.out.println("Unable to write path plan files");
				System.out.print(e.getMessage());
			}
    	}
    }
    public static void initialize()
    {
    	// Operator toggle here will cause files to be written
    	SmartDashboard.putBoolean("Write Paths", false);
    	
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
	    
	//***********
	    rightStartRightScaleTrajectory = new RobotTrajectory("rightStartRightScale");
	    rightStartRightScaleTrajectory.center = Pathfinder.generate(rightStartRightScalePath, config);

	    // We don't need to store the modifier persistently
	    modifier = new TankModifier(rightStartRightScaleTrajectory.center).modify(RobotMap.inch2Meter(RobotMap.WHEEL_TRACK_INCHES));
	    
	    // Extract the right and left trajectories
	    rightStartRightScaleTrajectory.left = modifier.getLeftTrajectory();
	    rightStartRightScaleTrajectory.right = modifier.getRightTrajectory();

	//***********
	    leftStartLeftScaleTrajectory = new RobotTrajectory("leftStartLeftScale");
	    leftStartLeftScaleTrajectory.center = Pathfinder.generate(leftStartLeftScalePath, config2);

	    // We don't need to store the modifier persistently
	    modifier = new TankModifier(leftStartLeftScaleTrajectory.center).modify(RobotMap.inch2Meter(RobotMap.WHEEL_TRACK_INCHES));
	    
	    // Extract the right and left trajectories
	    leftStartLeftScaleTrajectory.left = modifier.getLeftTrajectory();
	    leftStartLeftScaleTrajectory.right = modifier.getRightTrajectory();

	//***********
	    rightStartLeftScaleTrajectory = new RobotTrajectory("rightStartLefttScale");
	    rightStartLeftScaleTrajectory.center = Pathfinder.generate(rightStartLeftScalePath, config);

	    // We don't need to store the modifier persistently
	    modifier = new TankModifier(rightStartLeftScaleTrajectory.center).modify(RobotMap.inch2Meter(RobotMap.WHEEL_TRACK_INCHES));
	    
	    // Extract the right and left trajectories
	    rightStartLeftScaleTrajectory.left = modifier.getLeftTrajectory();
	    rightStartLeftScaleTrajectory.right = modifier.getRightTrajectory();

	//***********
	    leftStartRightScaleTrajectory = new RobotTrajectory("leftStartRightScale");
	    leftStartRightScaleTrajectory.center = Pathfinder.generate(leftStartRightScalePath, config);

	    // We don't need to store the modifier persistently
	    modifier = new TankModifier(leftStartRightScaleTrajectory.center).modify(RobotMap.inch2Meter(RobotMap.WHEEL_TRACK_INCHES));
	    
	    // Extract the right and left trajectories
	    leftStartRightScaleTrajectory.left = modifier.getLeftTrajectory();
	    leftStartRightScaleTrajectory.right = modifier.getRightTrajectory();
	    
	//********
	    leftStartMoveOnlyTrajectory = new RobotTrajectory("leftStartRightSwitchMoveOnly");
	    leftStartMoveOnlyTrajectory.center = Pathfinder.generate(leftStartMoveOnlyPath, config2);

	    modifier = new TankModifier(leftStartMoveOnlyTrajectory.center).modify(RobotMap.inch2Meter(RobotMap.WHEEL_TRACK_INCHES));
	    
	    leftStartMoveOnlyTrajectory.left = modifier.getLeftTrajectory();
	    leftStartMoveOnlyTrajectory.right = modifier.getRightTrajectory();
	    
	    
	 //********
	    rightStartMoveOnlyTrajectory = new RobotTrajectory("rightStartLeftSwitchMoveOnly");
	    rightStartMoveOnlyTrajectory.center = Pathfinder.generate(rightStartMoveOnlyPath, config2);
	    modifier = new TankModifier(rightStartMoveOnlyTrajectory.center).modify(RobotMap.inch2Meter(RobotMap.WHEEL_TRACK_INCHES));
	    
	    rightStartMoveOnlyTrajectory.left = modifier.getLeftTrajectory();
	    rightStartMoveOnlyTrajectory.right = modifier.getRightTrajectory();   
	 }

    // To trigger a call to the following function, create a Boolean object for the dashboard
    // that is polled in robot periodic or something like it, if the value becomes true,
    // set it back to false and then call this function.
    
   
    /**
     * writeTrajectoriesToFiles
     *  
	 * When called, will write each trajectory to a file
	 * Each RobotTrajectory instance has a name member that can be used as the file name
	 
	 * Each RobotTrajectory instance has a left and right member containing Segments
	 * An easier way to do this without needing to write a lot of code for each new trajectory is 
	 * to make an array of the above RobotTrajectory objects and simply loop through that array
	 * repeating the logic, below.
     * 
	 * Write the length as first value in file (integer then new line)
	 * Loop through the segments 
	 * 	Write dt, left.position, left.velocity, right.position, right.velocity (then new line)
	 * 
	 * Close file and move on to the next trajectory
     */
	public static void writeTrajectoriesToFiles() throws IOException
	{
		
		RobotTrajectory[] listofRobotTrajectories = 
		{
			testTrajectory0, 
			
			leftStartLeftSwitchTrajectory, 
			leftStartRightSwitchTrajectory, 
			leftStartLeftScaleTrajectory,
			leftStartRightScaleTrajectory, 
			leftStartMoveOnlyTrajectory, 

			centerStartLeftSwitchTrajectory, 
			centerStartRightSwitchTrajectory, 
			
			rightStartRightSwitchTrajectory, 
			rightStartLeftSwitchTrajectory,
			rightStartRightScaleTrajectory, 
			rightStartLeftScaleTrajectory, 
			rightStartMoveOnlyTrajectory
		};
		for(int a=0; a<listofRobotTrajectories.length; a++)
		{
			RobotTrajectory selectedTrajectory = listofRobotTrajectories[a];
			PrintWriter out
			= new PrintWriter(new BufferedWriter(new FileWriter(System.getProperty("user.home")+"/PathData/"+selectedTrajectory.name+".csv")));
			out.println(selectedTrajectory.center.length());
			for(int b=0; b<selectedTrajectory.center.length(); b++)
			{
				Trajectory.Segment segL = selectedTrajectory.left.get(b);
				Trajectory.Segment segR = selectedTrajectory.right.get(b);
				//segL.position + "," + segL.velocity + "," + segR.position + "," + segR.velocity+","+
				out.println((b+1)+","+segL.dt + "," + segL.x+","+segL.y+","+segR.x+","+segR.y);
			}
			out.close();
			System.out.println((a+1) + " of " + listofRobotTrajectories.length + "files completed");
		}		
	}

	public static RobotTrajectory getSelectedTrajectory() 
	{
		// A null trajectory means the robot will not move
		RobotTrajectory trajectory = null;
		
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
				
		CrossingMode crossingMode = crossingModeChooser.getSelected();
		
		Positions.GenericPositions switchPos = Robot.autonomousSubsystem.getSwitchPosition();
		Positions.GenericPositions scalePos = Robot.autonomousSubsystem.getScalePosition();
		
		// NOTE: NOTE: NOTE:
		// The following code is not pretty but clearly identifies the selection logic
		// At some point, a student is free to improve on the selection logic by using
		// referential techniques, tables, and such to traverse a series of choices.
		
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
				else // Primary is out of reach or unspecified, check for secondary scoring chance
				{
					System.out.println("SWITCH SOLUTION (PRIMARY) EXCLUDED");
					
					if (scalePos == Positions.GenericPositions.LEFT)
					{
						System.out.println("LEFT START LEFT SCALE (SECONDARY)");
						trajectory = PathPlans.leftStartLeftScaleTrajectory;
					}
					// In EXTREMELY unlikely cases the FMS could have an error on switch position tags
					// We will be complete with the logic and check for crossing... there is also
					// just as likely that this scale position data is corrupted as well, in which
					// case we will fall through
					else if (scalePos == Positions.GenericPositions.RIGHT && crossingMode == CrossingMode.ENABLE_CROSSING)
					{
						System.out.println("LEFT START RIGHT SCALE (SECONDARY)");
						trajectory = PathPlans.leftStartRightScaleTrajectory;						
					}
					else // We got here because crossing was disabled and all solution are on the other side of field
					{	 // or there was some error in the FMS and switch and scale sides are unknown
						System.out.println("NO SOLUTION: DRIVING FORWARD - CROSSING LINE");
					    trajectory = PathPlans.leftStartMoveOnlyTrajectory;
					} // end if scale secondary on left side start
				} // end if switch primary on left side start
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
					trajectory = PathPlans.leftStartMoveOnlyTrajectory;
					// NOTE: If FMS messes up, we won't move
					/// TODO: Consider Exchange?
				} // end if switch primary on center starts
			}
			else if (startingPositionChooser.getSelected() == StartingPosition.RIGHT)
			{
				if (switchPos == Positions.GenericPositions.RIGHT)
				{
					System.out.println("RIGHT START RIGHT SWITCH (PRIMARY)");
					trajectory = PathPlans.rightStartRightSwitchTrajectory;
				}
				// explicitly check for left in case FMS has bug
				else if (switchPos == Positions.GenericPositions.LEFT && crossingMode == CrossingMode.ENABLE_CROSSING)
				{
					System.out.println("RIGHT START LEFT SWITCH (PRIMARY)");
					trajectory = PathPlans.rightStartLeftSwitchTrajectory;
				}
				else  // Primary is out of reach or unspecified, check for secondary scoring chance
				{
					System.out.println("SWITCH SOLUTION (PRIMARY) EXCLUDED");
					
					if (scalePos == Positions.GenericPositions.RIGHT)
					{
						System.out.println("RIGHT START RIGHT SCALE (SECONDARY)");
						trajectory = PathPlans.rightStartRightScaleTrajectory;
					}
					// In EXTREMELY unlikely cases the FMS could have an error on switch position tags
					// We will be complete with the logic and check for crossing... there is also
					// just as likely that this scale position data is corrupted as well, in which
					// case we will fall through
					else if (scalePos == Positions.GenericPositions.LEFT && crossingMode == CrossingMode.ENABLE_CROSSING)
					{
						System.out.println("RIGHT START LEFT SCALE (SECONDARY)");
						trajectory = PathPlans.rightStartLeftScaleTrajectory;
						
					}
					else // We got here because crossing was disabled and all solution are on the other side of field
					{	 // or there was some error in the FMS and switch and scale sides are unknown
						System.out.println("NO SOLUTION: DRIVING FORWARD - CROSSING LINE");
					    trajectory = PathPlans.rightStartMoveOnlyTrajectory;
					} // end if scale secondary on right side start
				} // end if switch primary on right side start
			} // end if switch primary on any side start
			else 
			{
				System.out.println("Confused selecting switch");
				trajectory = PathPlans.rightStartMoveOnlyTrajectory;
			}
			break;
			
		case SCALE:
			if (startingPositionChooser.getSelected() == StartingPosition.LEFT)
			{
				if (scalePos == Positions.GenericPositions.LEFT)
				{
					System.out.println("LEFT START LEFT SCALE (PRIMARY)");
					trajectory = PathPlans.leftStartLeftScaleTrajectory;
				}
				// explicitly check for right in case FMS has bug
				else if (scalePos == Positions.GenericPositions.RIGHT && crossingMode == CrossingMode.ENABLE_CROSSING)
				{
					System.out.println("LEFT START RIGHT SCALE (PRIMARY)");
					trajectory = PathPlans.leftStartRightScaleTrajectory;
				}
				else // Primary is out of reach or unspecified
				{
					if (switchPos == Positions.GenericPositions.LEFT)
					{
						System.out.println("LEFT START LEFT SWITCH (SECONDARY)");
						trajectory = PathPlans.leftStartLeftSwitchTrajectory;
					}
					// In EXTREMELY unlikely cases the FMS could have an error on scale position tags
					// We will be complete with the logic and check for crossing... there is also
					// just as likely that this switch position data is corrupted as well, in which
					// case we will fall through
					else if (switchPos == Positions.GenericPositions.RIGHT && crossingMode == CrossingMode.ENABLE_CROSSING)
					{
						System.out.println("LEFT START RIGHT SWITCH (SECONDARY)");
						trajectory = PathPlans.leftStartRightSwitchTrajectory;	
					}
					else // We got here because crossing was disabled and all solution are on the other side of field
					{	 // or there was some error in the FMS and switch and scale sides are unknown
						System.out.println("NO SOLUTION: DRIVING FORWARD - CROSSING LINE");
					    trajectory = PathPlans.leftStartMoveOnlyTrajectory;
					} // end if switch positions secondary choices on left side start
				} // end if scale positions primary choices on left side start
			}
			else if (startingPositionChooser.getSelected() == StartingPosition.RIGHT)
			{
				if (scalePos == Positions.GenericPositions.RIGHT)
				{
					System.out.println("RIGHT START RIGHT SCALE (PRIMARY)");
					trajectory = PathPlans.rightStartRightScaleTrajectory;
				}
				// explicitly check for right in case FMS has bug
				else if (scalePos == Positions.GenericPositions.LEFT && crossingMode == CrossingMode.ENABLE_CROSSING)
				{
					System.out.println("RIGHT START LEFT SCALE (PRIMARY)");
					trajectory = PathPlans.rightStartLeftScaleTrajectory;
				}
				else // Primary is out of reach or unspecified
				{
					if (switchPos == Positions.GenericPositions.RIGHT)
					{
						System.out.println("RIGHT START RIGHT SWITCH (SECONDARY)");
						trajectory = PathPlans.rightStartRightSwitchTrajectory;
					}
					// In EXTREMELY unlikely cases the FMS could have an error on scale position tags
					// We will be complete with the logic and check for crossing... there is also
					// just as likely that this switch position data is corrupted as well, in which
					// case we will fall through
					else if (switchPos == Positions.GenericPositions.LEFT && crossingMode == CrossingMode.ENABLE_CROSSING)
					{
						System.out.println("RIGHT START LEFT SWITCH (SECONDARY)");
						trajectory = PathPlans.rightStartLeftSwitchTrajectory;	
					}
					else // We got here because crossing was disabled and all solution are on the other side of field
					{	 // or there was some error in the FMS and switch and scale sides are unknown
						System.out.println("NO SOLUTION: DRIVING FORWARD - CROSSING LINE");
					    trajectory = PathPlans.rightStartMoveOnlyTrajectory;
					} // end if switch position secondary choices on right side start
				} // end if scale positions primary choices on right side start		
			} // end if scale primary on either side start
			else 
			{
				System.out.println("Confused starting scale");
				trajectory = PathPlans.rightStartMoveOnlyTrajectory;
			}
			break;
			
		case EXCHANGE:
		case CROSS_LINE: //Done purposely
		default:
			if (startingPositionChooser.getSelected() == StartingPosition.LEFT)
			{
				System.out.println("CROSSING LINE ON LEFT SIDE");
				trajectory = PathPlans.leftStartMoveOnlyTrajectory;
			}
			else
			{
				System.out.println("CROSSING LINE ON RIGHT SIDE");
				trajectory = PathPlans.rightStartMoveOnlyTrajectory;
			}
			// else center and we don't move
			/// TODO: Maybe we should, but better negotiation with alliance is prefered
			break;
		}
		
		return trajectory;
		
	}

}