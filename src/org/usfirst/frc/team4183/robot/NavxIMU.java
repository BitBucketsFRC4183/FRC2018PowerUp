package org.usfirst.frc.team4183.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class NavxIMU {
	
	private final AHRS ahrs;
	
	// 60 Hz is the default, we're just giving it the default,
	// but we need to know this number to compute yawRate.
	private final int AHRS_UPDATE_RATE = 60;  
	private final boolean DEBUG_THREAD = false;
	
	NavxIMU() {
		
		System.out.println( "Starting NavX AHRS");				
		ahrs = new AHRS(SPI.Port.kMXP, (byte)AHRS_UPDATE_RATE);

		// Wait a bit in background, the print connected & firmware info
		new Thread() {
			public void run() {
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {}
				System.out.format("NavX isConnected=%b firmware=%s\n", 
						ahrs.isConnected(), ahrs.getFirmwareVersion());
			}
		}.start();
		
		
		// Start thread to print out something at a reasonable rate (testing)
		if( DEBUG_THREAD) {
			new Thread() { 
				public void run() {
					while(true) {
						try {
							Thread.sleep(500);
						} catch (InterruptedException e) {}	
						System.out.format("isConnected=%b isCalibrating=%b Yaw=%.2f Rate=%.2f\n", 
								isConnected(), isCalibrating(), getYawDeg(), getYawRateDps());						
					}
				}
			}.start();
		}				
	}
	
	
	// Return yaw angle according to right-hand-rule with z-axis up;
	// that is, +yaw is CCW looking down on robot.	
	public synchronized double getYawDeg() {
		
		if( !isConnected()) {
			System.err.println( "Error, Yaw requested but NavX not connected");
			return 0.0;
		}
		
		if( isCalibrating()) {
			System.err.println( "Warning, Yaw requested but NavX is calibrating");
		}
		
		// Need the - sign to get the Navx to agree with the yaw definition
		return -ahrs.getAngle();
	}

	public synchronized double getYawRateDps() {
		
		if( !isConnected()) {
			System.err.println( "Error, Rate requested but NavX not connected");
			return 0.0;
		}
		
		if( isCalibrating()) {
			System.err.println( "Warning, Rate requested but NavX is calibrating");
		}
		
		// The NavX getRate() does NOT return degrees per second (as claimed);
		// it actually returns just the difference between two successive yaw values in degrees
		// (so it actually returns degrees per delta-T).
		// They forgot to multiply by their own internal update rate
		// (60 HZ by default). So we do it here.
		// Need the - sign to get the Navx to agree with the yaw definition.		
		return -AHRS_UPDATE_RATE*ahrs.getRate();
	}
	
	public synchronized double getFwdAccel_G() {
		
		if( !isConnected()) {
			System.err.println( "Error, Accel requested but NavX not connected");
			return 0.0;
		}
		
		if( isCalibrating()) {
			System.err.println( "Warning, Accel requested but NavX is calibrating");
		}
		
		return ahrs.getRawAccelX();
	}
	
	public synchronized boolean isConnected() {
		return ahrs.isConnected();
	}
	
	public synchronized boolean isCalibrating() {
		return ahrs.isCalibrating();
	}

}
