package org.usfirst.frc.team4183.utils;

public class RateLimit {
	
	private final double maxChangePerMsec;
	private double prev = 0.0;
	private double prevMsec;
	
	/**
	 * Constructor
	 * @param maxChangePerSecond units are per second
	 */
	public RateLimit( double maxChangePerSecond) {
		this.maxChangePerMsec = Math.abs(maxChangePerSecond)/1000.0;
		prevMsec = System.currentTimeMillis();
	}
	
	public double f( double x) {

		double absx = Math.abs(x); 
		double sgnx = Math.signum(x); 

		double delta = maxChangePerMsec * (System.currentTimeMillis() - prevMsec);     
		if( absx > prev + delta ) 
			absx = prev + delta; 
		
		prev = absx; 
		prevMsec = System.currentTimeMillis(); 

		return sgnx * absx;
	}
}
