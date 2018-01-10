package org.usfirst.frc.team4183.utils;

public class SettledDetector{
	
	private volatile long tlast;
	private final long msecs; 
	private final double allowedError; 
	
	public SettledDetector(long msecs, double allowedError) {
		this.msecs = msecs;
		this.allowedError = allowedError;
		tlast = System.currentTimeMillis();
	}
	
	public void set(double error) {
		if (Math.abs(error) >= allowedError) {
			tlast = System.currentTimeMillis();
		}		
	}
	
	public boolean isSettled() {
		return (System.currentTimeMillis() - tlast) > msecs; 
	}
}
