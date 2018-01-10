package org.usfirst.frc.team4183.utils;

public class ZeroCrossDetector {
	
	double prev = Double.NaN;
	
	public boolean setGet( double val) {
		boolean rtn = false;
		if( !Double.isNaN(prev) 
			&& 
			(Math.signum(prev) * Math.signum(val) < 0)
		) {			
				rtn = true;
		}
		
		prev = val;
		return rtn;
	}
}
