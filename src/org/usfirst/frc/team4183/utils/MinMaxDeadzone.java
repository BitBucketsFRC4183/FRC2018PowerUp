package org.usfirst.frc.team4183.utils;

public class MinMaxDeadzone {
	
	private double minDrive;
	private final double maxDrive, deadZone;
	
	/**
	 * Define parameters for y = f(x) below
	 * @param deadZone If |error| < deadZone then y = 0
	 * @param min  Smallest y returned (except for deadzone case)
	 * @param max  Largest y returned 
	 */
	public MinMaxDeadzone( double deadZone, double min, double max) {		
		this.deadZone = deadZone;
		this.minDrive = min;
		this.maxDrive = max;
	}
	
	
	/**
	 * y = f(x)
	 * @param x  
	 * @param error  Loop error (for deadzone determination)
	 * @return y
	 */
	public double f( double x, double error) {

		if( Math.abs(error) < deadZone) 
			return 0.0;

		double sign = Math.signum(x);
		x = Math.abs(x);						
		if( x > maxDrive) x = maxDrive;
		if( x < minDrive) x = minDrive;
		
		return sign*x;						
	}
}
