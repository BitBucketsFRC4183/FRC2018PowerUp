package org.usfirst.frc.team4183.utils;

import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.Trajectory;

public class RobotTrajectory
{
	public String name;
	public Trajectory left;
	public Trajectory center;
	public Trajectory right;
	public int runCount;		// How many time this trajectory is allowed to run (0 stops it)
	
	public RobotTrajectory()
	{
		name = "NONE";
		runCount = 0; // Until otherwise specified
	}
	public RobotTrajectory(String aName)
	{
		name = aName;
		runCount = 1; // Until otherwise specified
		
		System.out.println("Time: " + Timer.getFPGATimestamp() + ": Creating Trajectory: " + name);
	}
}
